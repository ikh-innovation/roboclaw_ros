#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from std_srvs.srv import SetBool
from ikh_ros_msgs.msg import FloatStamped
from ikh_ros_msgs.msg import MovePrismaticAction, MovePrismaticFeedback, MovePrismaticResult, MovePrismaticGoal
from std_msgs.msg import String, Float32MultiArray
from roboclaw_ros.roboclaw_driver import Roboclaw
import numpy as np
import re, csv, os, threading, time
import datetime as  dt
from enum import Enum

# class CurrentDataLogger:
#     """
#     Class for logging motor current data to CSV files for later analysis.
    
#     Helps detect mechanical limits by capturing currents behavior
#     between deck positions (up/down).
#     """
#     def __init__(self, log_dir="/tmp/roboclaw_current_logs"):
#         self.log_dir = log_dir
#         self.is_logging = False
#         self.log_file = None
#         self.csv_writer = None
#         self.csv_file = None
#         self.start_time = None
#         self.movement_type = None
#         self.lock = threading.Lock()  # Add a lock for thread safety
        
#         # Create log directory if it doesn't exist
#         if not os.path.exists(self.log_dir):
#             os.makedirs(self.log_dir)
    
#     def start_logging(self, movement_type):
#         """
#         Start logging current data for a specific movement type.
        
#         Args:
#             movement_type (str): Type of movement ('up_to_down' or 'down_to_up')
#         """
#         with self.lock:  # Use lock for thread safety
#             if self.is_logging:
#                 self.stop_logging()
                
#             self.movement_type = movement_type
#             timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
#             self.log_file = os.path.join(self.log_dir, "current_log_%s_%s.csv" % (movement_type, timestamp))
#             self.csv_file = open(self.log_file, 'w')
#             self.csv_writer = csv.writer(self.csv_file)
            
#             # Write header with buffer status column
#             self.csv_writer.writerow(['timestamp', 'elapsed_time', 'm1_current', 'm2_current', 
#                                 'm1_current_last', 'm2_current_last', 'output_power', 'buffer_ready'])
#             self.start_time = rospy.Time.now().to_sec()
#             self.is_logging = True
#             rospy.loginfo("Started logging current data for %s movement to %s" % (movement_type, self.log_file))

#     def log_data(self, m1_current, m2_current, m1_current_last, m2_current_last, output_power, buffer_ready=False):
#         """
#         Log a data point to the CSV file.
        
#         Args:
#             m1_current (float): Mean current for motor 1
#             m2_current (float): Mean current for motor 2
#             m1_current_last (float): Latest current for motor 1
#             m2_current_last (float): Latest current for motor 2
#             output_power (float): Output power of the motors
#             buffer_ready (bool): Whether the buffer contains valid data
#         """
#         with self.lock:  # Use lock for thread safety
#             if not self.is_logging or self.csv_writer is None:
#                 return
                
#             timestamp = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
#             elapsed_time = rospy.Time.now().to_sec() - self.start_time
            
#             self.csv_writer.writerow([timestamp, elapsed_time, m1_current, m2_current, 
#                                 m1_current_last, m2_current_last, output_power, buffer_ready])
    
#     def stop_logging(self):
#         """Stop the current logging session and close the file."""
#         with self.lock:  # Use lock for thread safety
#             if self.is_logging and self.csv_file is not None:
#                 self.is_logging = False  # Set this first to prevent new writes
#                 self.csv_file.close()
#                 rospy.loginfo("Stopped logging current data. Log saved to %s" % self.log_file)
#                 self.csv_file = None
#                 self.csv_writer = None
            
class MotorCurrents:
    def __init__(self):
        self._bufferSize =  rospy.get_param("~currents_buffer_size", 10)  # Size of the buffer
        self.buffer_filled = False  # Track if buffer is filled
        self.buffer_fill_counter = 0  # Count of real measurements added
        self.m1 = np.array([0.0 for i in range(self._bufferSize)])
        self.m2 = np.array([0.0 for i in range(self._bufferSize)])
        self.prevMeans = [0.0, 0.0]
        self._alpha = rospy.get_param("~exp_filter_alpha", 0.2)
        self._m1_exp = 0.0
        self._m2_exp = 0.0
        self._exp_initialized = False
        self.data_lock = threading.Lock()  # Add thread safety
    
    def reset(self):
        """Reset all current buffers to zero values."""
        with self.data_lock:
            self.m1 = np.array([0.0 for i in range(self._bufferSize)])
            self.m2 = np.array([0.0 for i in range(self._bufferSize)])
            self.prevMeans = [0.0, 0.0]
            self.buffer_filled = False
            self.buffer_fill_counter = 0
            rospy.logdebug("Current buffers reset to zero")
    
    def appendM1(self, value):
        with self.data_lock:
            self.m1 = np.append(self.m1, value)
            self.m1 = np.delete(self.m1, 0)
            self._update_buffer_status()
            self._update_buffer_status()
            # Exponential filter on last value
            if not self._exp_initialized:
                self._m1_exp = value
                self._exp_initialized = True
            else:
                self._m1_exp = self._alpha * value + (1 - self._alpha) * self._m1_exp

    def appendM2(self, value):
        with self.data_lock:
            self.m2 = np.append(self.m2, value)
            self.m2 = np.delete(self.m2, 0)
            self._update_buffer_status()
            # Exponential filter on last value
            if not self._exp_initialized:
                self._m2_exp = value
                self._exp_initialized = True
            else:
                self._m2_exp = self._alpha * value + (1 - self._alpha) * self._m2_exp
    
    def _update_buffer_status(self):
        """Track when buffer is completely filled with real readings"""
        if not self.buffer_filled:
            self.buffer_fill_counter += 0.5  # Increment by 0.5 (both M1+M2 make 1.0)
            if self.buffer_fill_counter >= self._bufferSize:
                self.buffer_filled = True
                rospy.logdebug("Current buffer fully populated with real data")
    
    def is_buffer_ready(self):
        """Returns True if buffer filled with real measurements"""
        with self.data_lock:
            return self.buffer_filled

    def getMeanM1M2Values(self):
        with self.data_lock:
            return [np.mean(self.m1), np.mean(self.m2)]

    def getMeanM1M2Derivatives(self, dt):
        with self.data_lock:
            now = self.getMeanM1M2Values()
            res = self.getDerivatives(now, self.prevMeans, dt)
            self.prevMeans = now
            return res

    def getDerivatives(self, now, prev, dt):
        return (np.array(now)-np.array(prev))/dt

    def getMaxOfMeans(self):
        with self.data_lock:
            return np.max(self.getMeanM1M2Values())

    def getM1(self):
        with self.data_lock:
            return self.m1.copy()  # Return copy to prevent race conditions

    def getM2(self):
        with self.data_lock:
            return self.m2.copy()  # Return copy to prevent race conditions

    def getOutputPower(self):
        with self.data_lock:
            return (np.mean(self.m1)**2) + (np.mean(self.m2)**2)
    
    def getExpFilteredCurrents(self):
        with self.data_lock:
            return [self._m1_exp, self._m2_exp]

    def getExpFilteredOutputPower(self):
        with self.data_lock:
            return (self._m1_exp ** 2) + (self._m2_exp ** 2)
        
class DeckState(Enum):
    UNDEFINED = -1
    UP = 1
    DOWN = 0

class Node:
    def __init__(self):
        self.serial_lock = threading.Lock()
        # rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        self.dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        self.baud_rate = int(rospy.get_param("~baud", "115200"))
        self.address = int(rospy.get_param("~address", "128"))
        self.firmware_version = rospy.get_param("~version", "4.2.8") #TODO: add param in config

        rospy.sleep(2)
        # Check port permissions
        if (self._port_permissions(self.dev_name)):
            rospy.loginfo("Port permissions are acceptable.")
        else:
            rospy.logwarn(
                "Port permissions are not valid. (sudo chmod 777 [port name])")
            rospy.logwarn("Try to give permissions...")
            if (not self._give_port_permissions(self.dev_name)):
                rospy.logerr("cannot achieved")
                exit(-1)
            rospy.loginfo("Permissions are now ok!")
        rospy.sleep(2)
        # Create a roboclaw instance
        self.roboclaw = Roboclaw(self.dev_name, self.baud_rate,timeout=0.02,retries=1)
        # Open roboclaw port and check version
        self._open_roboclaw_port()
        self.motorCurrents = MotorCurrents()
        
        # Publishers
        self._m1_current_last_pub = rospy.Publisher('m1_current_last', FloatStamped, queue_size=10)
        self._m2_current_last_pub = rospy.Publisher('m2_current_last', FloatStamped, queue_size=10)
        self._m1_current_pub = rospy.Publisher('m1_current_mean', FloatStamped, queue_size=10)
        self._m2_current_pub = rospy.Publisher('m2_current_mean', FloatStamped, queue_size=10)
        self._output_power = rospy.Publisher('output_power', FloatStamped, queue_size=10)
        self._deck_position_pub = rospy.Publisher('deck_position', String, queue_size=10, latch=True)
        self._status_pub = rospy.Publisher('status', String, queue_size=5)
        self._temp_pub = rospy.Publisher('temperature', FloatStamped, queue_size=5)
        self._pwms = rospy.Publisher('pwm', Float32MultiArray, queue_size=5)

        # Service Server
        self.deck_control_srv = rospy.Service('move_prismatic', SetBool, self._deck_control_cb)
        
        # Action Server
        self.action_server = SimpleActionServer('move_prismatic_action',
            MovePrismaticAction,
            execute_cb=self._execute_action_cb,
            auto_start=False
        )

        # Parameters
        self._stop_move_timeout = rospy.get_param("~stop_move_timeout", 20) #20
        self._stop_with_time = rospy.get_param("~stop_with_time",False)
        self._stop_with_time_seconds = rospy.get_param("~stop_with_time_seconds",10.0)
        self._power_stop_threshold = rospy.get_param("~power_stop_threshold", 0.3) #0.3
        self._max_cnt_to_stop = rospy.get_param("~max_counter_stop", 5) #5
        self._pwm_duty_cycle = rospy.get_param("~pwm_duty_cycle", 65) #65
        self._publish_roboclaw_temperature = rospy.get_param("~publish_temperature", True)
        self._publish_roboclaw_status = rospy.get_param("~publish_status", True)
        self._publish_currents = rospy.get_param("~publish_currents", True)
        self._inverted_logic = rospy.get_param("~service_inverted_logic",False)
        self._ce_block_enabled = rospy.get_param("mower/ce_block_enabled",False)
        self._emergency_enabled = rospy.get_param("mower/emergency_stop",False)
        update_rate = rospy.get_param("~rate",10.0)
        publish_rate = rospy.get_param("~publish_rate",5.0)
        
        self._is_running = False # Parameter to indicate that the motors are running.
        self._deck_state = DeckState.UNDEFINED
        self._update_deck_state(self._deck_state)
        self._update_period = rospy.Duration().from_sec(1.0/update_rate)
        self._publish_period = rospy.Duration().from_sec(1.0/publish_rate)
       
        # Create the current data logger
        # log_dir = rospy.get_param("~log_dir", os.path.join(os.path.expanduser("~"), "roboclaw_logs"))
        # self.current_logger = CurrentDataLogger(log_dir=log_dir)
            
    def _read_data_cb(self, timer):
        # Read and append currents to the current instance
        #tick = rospy.Time.now().to_sec()
        self.serial_lock.acquire()
        res = self.read_currents()
        if(self.serial_lock.locked()):
            self.serial_lock.release()
        if res is not None:
            m1_raw, m2_raw = res
            # Convert to amps
            m1_current = m1_raw / 100.0
            m2_current = m2_raw / 100.0

            #print("Time to read Currents: {}".format(rospy.Time.now().to_sec()-tick))
            self.motorCurrents.appendM1(m1_current)
            self.motorCurrents.appendM2(m2_current)
        
            # Log data if logging is active
            # self._log_current_data()
  
    def _publish_data_cb(self, timer):
        # Publish Mean Current Messages
        if (self._publish_currents and (self._m1_current_pub.get_num_connections()>0 or self._m2_current_pub.get_num_connections()>0)):
            # Publish mean currents
            mean_currents = self.motorCurrents.getMeanM1M2Values()
            msg = FloatStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = mean_currents[0]
            self._m1_current_pub.publish(msg)
            msg.data = mean_currents[1]
            self._m2_current_pub.publish(msg)
            # Publish output power
            output_power = self.motorCurrents.getOutputPower()
            msg.data = output_power
            self._output_power.publish(msg)
            # Publish last currents
            current_1_last = self.motorCurrents.getM1()
            current_2_last = self.motorCurrents.getM2()
            msg.data = current_1_last[-1]
            self._m1_current_last_pub.publish(msg)
            msg.data = current_2_last[-1]
            self._m2_current_last_pub.publish(msg)   
            
        # Read and publish Errors
        self.serial_lock.acquire()
        if (self._publish_roboclaw_status and self._status_pub.get_num_connections()>0):
            res = self.read_list_of_errors()
            if (res!=None):
                self._publish_list_of_errors(res)
        
        # Read and publish Temp
        if (self._publish_roboclaw_temperature and self._temp_pub.get_num_connections()>0):
            res = self.read_temps()
            if (res!=None):
                self._publish_temperature(res/10)
        
        if (self._pwms.get_num_connections()>0):
            pwms = self.roboclaw.ReadPWMs(self.address)
            if (pwms[0]):
                msg = Float32MultiArray()
                #tick = rospy.Time.now().to_sec()
                #print("Time to read PWMs: {}".format(rospy.Time.now().to_sec()-tick))
                msg.data.append(pwms[1])
                msg.data.append(pwms[2])
                self._pwms.publish(msg)
        
        if(self.serial_lock.locked()):
            self.serial_lock.release()
                      
    def _open_roboclaw_port(self):
        rospy.loginfo('Roboclaw Node: Try to open port...')
        try:
            self.roboclaw.Open()
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw")
            rospy.logdebug(e)
        rospy.loginfo("Roboclaw Node: Try to read version...")
        try:
            version = self.roboclaw.ReadVersion(self.address)
        except Exception as e:
            rospy.logwarn("Problem getting roboclaw version")
            rospy.logdebug(e)
            pass
        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
            self.firmware_version = "4.2.8"  # Default to old version #TODO: decide if it is redundant bcs of conf param
        else:
            rospy.logdebug(repr(version[1]))
            # Regex extraction of version number
            version_str = version[1].strip()  # Removes trailing newline
            match = re.search(r'v(\d+\.\d+\.\d+)', version_str)
            if match:
                self.firmware_version = match.group(1)
                rospy.loginfo("Roboclaw version: %s" % self.firmware_version)
            else:
                rospy.logwarn("Could not parse version number from response")
          
    def _port_permissions(self, port):
        try:
            import os
            if (os.access(port, os.R_OK) and os.access(port, os.W_OK) and os.access(port, os.X_OK)):
                return True
            else:
                return False
        except:
            rospy.logerror('Roboclaw: Cannot read port permissions')

    def _give_port_permissions(self, port):
        import os
        if os.path.exists(port):
            os.system('echo INnovation! | sudo -S chmod 777 '+port)
            return True
        else:
            return False

    def read_list_of_errors(self):
        try:
            #tick = rospy.Time.now().to_sec()
            lst = self.roboclaw.ReadErrorDecoded(self.address)
            #print("Time to read Status: {}".format(rospy.Time.now().to_sec()-tick))
            if (lst!=None):
                return lst
            else:
                return None
        except:
            raise Exception("Cannot read roboclaw errors")

    def read_temps(self):
        try:
            temp1, temp2 = self.roboclaw.ReadTemp(self.address)
            if (temp1):
                return temp2
            else:
                return None
        except:
            raise Exception("Cannot read roboclaw errors")
    
    def read_currents(self):
        try:
            _cr, m1_current, m2_current = self.roboclaw.ReadCurrents(
            self.address)
            if (_cr):
                return (m1_current, m2_current)
            else:
                return None
        except:
            raise Exception("Cannot read roboclaw motor currents")
        
    def _publish_list_of_errors(self, lst):
        msg = String()
        msg.data = str(lst)
        self._status_pub.publish(msg)
    
    def _publish_temperature(self, temp):
        msg = FloatStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = temp
        self._temp_pub.publish(msg)

    def convert_pwm_to_duty(self, old_pwm):
        """
        Converts a PWM duty cycle from the 0-127 scale (used in RoboClaw compatibility commands )
        to the 0 to +32767 scale used in RoboClaw signed Duty commands ).

        Args:
            old_pwm (int): Original PWM value in the range 0-127.

        Returns:
            int: Duty cycle value in the range -32767 to +32767.
        """
        duty = int(float(old_pwm) * 32767 / 127)
        return duty
    
    def compare_versions(self, ver1, ver2):
        """
        Compare two version strings in semantic versioning format (e.g., "4.3.6").
        
        Args:
            ver1 (str): First version string to compare.
            ver2 (str): Second version string to compare.
            
        Returns:
            int: -1 if ver1 < ver2, 0 if ver1 == ver2, 1 if ver1 > ver2
        """
        v1_parts = [int(x) for x in ver1.split('.')]
        v2_parts = [int(x) for x in ver2.split('.')]
        
        # Pad shorter version with zeros
        while len(v1_parts) < len(v2_parts):
            v1_parts.append(0)
        while len(v2_parts) < len(v1_parts):
            v2_parts.append(0)
            
        for i in range(len(v1_parts)):
            if v1_parts[i] < v2_parts[i]:
                return -1
            elif v1_parts[i] > v2_parts[i]:
                return 1
                
        return 0  # Versions are equal

    def _use_duty_commands(self):
        """
        Determines if the firmware version supports Duty commands.
        
        Firmware versions >= 4.3.6 use DutyM1/DutyM2 commands which accept values
        in the range -32767 to +32767. Older firmware versions use ForwardM1/BackwardM1
        commands which accept values in the range 0-127.
        
        Returns:
            bool: True if the firmware supports Duty commands, False otherwise.
        """
        return self.compare_versions(self.firmware_version, '4.3.6') >= 0

    def _deck_control_cb(self, req):
        try:
            # Invert logic if necessary (inverted is true for up and false for down)
            if not self._inverted_logic:
                req.data = not req.data
            
            # Create an action client
            client = SimpleActionClient('move_prismatic_action', MovePrismaticAction)

            # Wait for the action server to be available
            client.wait_for_server(timeout=rospy.Duration(5.0))  # 5-second timeout

            # Send the goal
            goal = MovePrismaticGoal()
            goal.position = req.data
            client.send_goal(goal)

            # Wait for the result with a timeout
            if not client.wait_for_result(timeout=rospy.Duration(20.0)):  # 20-second timeout
                rospy.logerr("Action server did not respond in time")
                return (False, "Action server timeout")

            # Get the result
            result = client.get_result()

            # Return the result as a service response
            return (result.success, result.message)

        except rospy.ROSException as e:
            rospy.logerr("ROS Exception in deck_control_cb: %s" % str(e))
            return (False, "ROS Exception: %s" % str(e))
        except Exception as e:
            rospy.logerr("Unexpected error in deck_control_cb: %s" % str(e))
            return (False, "Unexpected error: %s" % str(e))

    def motors_forward(self, motor_1_speed, motor_2_speed):
        self.serial_lock.acquire()
        try:
            self.roboclaw.DutyM1(self.address, motor_1_speed)
            self.roboclaw.DutyM2(self.address, motor_2_speed)
        finally:
            if self.serial_lock.locked():
                self.serial_lock.release()

    def motors_backward(self, motor_1_speed, motor_2_speed):
        self.serial_lock.acquire()
        try:
            self.roboclaw.DutyM1(self.address, -motor_1_speed)
            self.roboclaw.DutyM2(self.address, -motor_2_speed)
        finally:
            if self.serial_lock.locked():
                self.serial_lock.release()
    
    def motors_stop(self):
        self.serial_lock.acquire()
        self._is_running = False
        try:
            self.roboclaw.DutyM1(self.address, 0)
            self.roboclaw.DutyM2(self.address, 0)
            rospy.sleep(0.1)
            self.roboclaw.DutyM1(self.address, 0)
            self.roboclaw.DutyM2(self.address, 0)
            rospy.sleep(0.1)
            self.roboclaw.DutyM1(self.address, 0)
            self.roboclaw.DutyM2(self.address, 0)
        finally:
            if self.serial_lock.locked():
                self.serial_lock.release()

    def _update_deck_state(self, cmd):
        rospy.loginfo("Updating deck state to: %s" % cmd.name) 
        msg = String()
        if cmd == DeckState.UNDEFINED:
            self._deck_state = DeckState.UNDEFINED
            rospy.set_param("deck/state", DeckState.UNDEFINED.value)
            msg.data = cmd.name.lower()
        elif cmd == DeckState.DOWN:
            self._deck_state = DeckState.DOWN
            rospy.set_param("deck/state",  DeckState.DOWN.value)
            msg.data = cmd.name.lower()
        elif cmd == DeckState.UP:
            self._deck_state = DeckState.UP
            rospy.set_param("deck/state", DeckState.UP.value)
            msg.data = cmd.name.lower()
        self._deck_position_pub.publish(msg)

    def _execute_action_cb(self, goal):
        feedback = MovePrismaticFeedback()
        result = MovePrismaticResult()

        try:            
            cmd = DeckState.UP if goal.position else DeckState.DOWN
            time_started = rospy.Time.now().to_sec()
            cnt = 0

            if cmd == self._deck_state:
                rospy.logwarn("Deck already in this position. Not moving.")
                result.success = True
                result.message = "Deck already in this position."
                self.action_server.set_succeeded(result)
                return

            # Start logging based on movement direction
            # if cmd == DeckState.UP:
            #     self.current_logger.start_logging("down_to_up")
            # else:
            #     self.current_logger.start_logging("up_to_down")
            
            rospy.logwarn("- Deck goes to %s position" % ("lower" if cmd == DeckState.DOWN else "upper"))
            self._is_running = True

            while not rospy.is_shutdown():
                # Check Check if emergency stop is enabled
                # if self._emergency_enabled:
                #     rospy.logwarn("Emergency stop is enabled! Cannot execute deck action")
                #     result.success = False
                #     result.message = "Emergency stop is enabled!"
                #     self.action_server.set_aborted(result)
                #     return
                
                # # Check if CE block is enabled 
                # if self._ce_block_enabled:
                #     rospy.logwarn("CE block is enabled! Cannot execute deck action")
                #     result.success = False
                #     result.message = "CE block is enabled!"
                #     self.action_server.set_aborted(result)
                #     return
            
                # Check for preemption
                if self.action_server.is_preempt_requested():
                    rospy.logwarn("Preemption requested. Stopping the operation.")
                    self.motors_stop()  # Stop the motors
                    # self.current_logger.stop_logging()  
                    # Update the deck state to undefined
                    self._update_deck_state(DeckState.UNDEFINED)
                    result.success = False
                    result.message = "Operation preempted by another goal."
                    self.action_server.set_preempted(result)
                    self._is_running = False
                    return

                duration = rospy.Time.now().to_sec() - time_started

                # Continuously send motor commands
                if cmd == DeckState.DOWN:
                    converted_pwm_duty_1 = self.convert_pwm_to_duty(int(self._pwm_duty_cycle))
                    converted_pwm_duty_2 = self.convert_pwm_to_duty(self._pwm_duty_cycle)
                    self.motors_backward(converted_pwm_duty_1, converted_pwm_duty_2)
                else:  # UP
                    converted_pwm_duty_1 = self.convert_pwm_to_duty(int(self._pwm_duty_cycle))
                    converted_pwm_duty_2 = self.convert_pwm_to_duty(self._pwm_duty_cycle)
                    self.motors_forward(converted_pwm_duty_1, converted_pwm_duty_2)
            
                exp_output_power = self.motorCurrents.getExpFilteredOutputPower()
                
                # Publish feedback
                feedback.power_output = exp_output_power
                feedback.iterations = cnt
                feedback.duration = duration
                self.action_server.publish_feedback(feedback)

                # Stop if timeout is reached
                if duration > self._stop_move_timeout:
                    self.motors_stop()
                    # self.current_logger.stop_logging()  
                    self._update_deck_state(DeckState.UNDEFINED)
                    result.success = False
                    result.message = "Timeout reached!"
                    self.action_server.set_aborted(result)
                    return

                # Stop with time
                elif self._stop_with_time and duration >= self._stop_with_time_seconds:
                    rospy.sleep(1)
                    self.motors_stop()
                    # self.current_logger.stop_logging()  
                    self._update_deck_state(cmd)
                    result.success = True
                    result.message = "Position reached."
                    self.action_server.set_succeeded(result)
                    return

                # Stop based on power threshold
                elif duration > 2 and exp_output_power < self._power_stop_threshold and not self._stop_with_time:
                    cnt += 1
                    if cnt > self._max_cnt_to_stop:
                        rospy.sleep(1)
                        self.motors_stop()
                        # self.current_logger.stop_logging()  
                        self._update_deck_state(cmd)
                        result.success = True
                        result.message = "Position reached."
                        self.action_server.set_succeeded(result)
                        return

                rospy.sleep(self._update_period.to_sec())

            # If shutdown is triggered
            self.motors_stop()
            # self.current_logger.stop_logging()  
            self._update_deck_state(DeckState.UNDEFINED)
            result.success = False
            result.message = "Operation interrupted by shutdown."
            self.action_server.set_aborted(result)

        except Exception as e:
            rospy.logerr("Unexpected error in execute_action_cb: %s" % str(e))
            self.motors_stop()
            # self.current_logger.stop_logging()  
            self._update_deck_state(DeckState.UNDEFINED)
            result.success = False
            result.message = "Unexpected error: %s" % str(e)
            self.action_server.set_aborted(result)

    def _log_current_data(self):
        """Log the current motor data if logging is active."""
        if self.current_logger.is_logging:
            mean_currents = self.motorCurrents.getMeanM1M2Values()
            last_currents = [self.motorCurrents.getM1()[-1], self.motorCurrents.getM2()[-1]]
            output_power = self.motorCurrents.getOutputPower()
            buffer_ready = self.motorCurrents.is_buffer_ready()
            self.current_logger.log_data(
                mean_currents[0], mean_currents[1],
                last_currents[0], last_currents[1],
                output_power,
                buffer_ready
            )
            
    def run(self):
        rospy.loginfo("Starting roboclaw node")
        # Start the action server
        self.action_server.start()
        time.sleep(1)
        self.timer1 = rospy.Timer(self._update_period, self._read_data_cb)
        self.timer2 = rospy.Timer(self._publish_period, self._publish_data_cb)
        rospy.spin()
        
if __name__ == "__main__":
    try:
        rospy.init_node("roboclaw_node")
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
