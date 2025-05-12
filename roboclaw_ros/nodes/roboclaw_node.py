#!/usr/bin/env python
import rospy
from ikh_ros_msgs.msg import FloatStamped
from actionlib import SimpleActionClient, SimpleActionServer
from ikh_ros_msgs.msg import MovePrismaticAction, MovePrismaticFeedback, MovePrismaticResult, MovePrismaticGoal
from std_srvs.srv import SetBool
from std_msgs.msg import String, Float32MultiArray
import numpy as np
from roboclaw_ros.roboclaw_driver import Roboclaw

from threading import Lock
from enum import Enum
class MotorCurrents:
    def __init__(self):
        self._bufferSize = 10
        self.m1 = np.array([0.0 for i in range(self._bufferSize)])
        self.m2 = np.array([0.0 for i in range(self._bufferSize)])
        self.prevMeans = [0.0, 0.0]

    def setBufferSize(self, size):

        if (self._bufferSize == size):
            pass
        elif (self._bufferSize > size):
            self._bufferSize = size
            np.insert(self.m1, 0, np.zeros(self._bufferSize - self.m1.size))
            np.insert(self.m2, 0, np.zeros(self._bufferSize - self.m2.size))
        else:
            self._bufferSize = size
            self.m1 = self.m1[self.m1.size-self._bufferSize:self.m1.size]
            self.m2 = self.m2[self.m2.size-self._bufferSize:self.m2.size]

    def appendM1(self, value):
        self.m1 = np.append(self.m1, value)
        self.m1 = np.delete(self.m1, 0)

    def appendM2(self, value):
        self.m2 = np.append(self.m2, value)
        self.m2 = np.delete(self.m2, 0)

    def getMeanM1M2Values(self):
        return [np.mean(self.m1), np.mean(self.m2)]

    def getMeanM1M2Derivatives(self, dt):
        now = self.getMeanM1M2Values()
        res = self.getDerivatives(now, self.prevMeans, dt)
        self.prevMeans = now
        return res

    def getDerivatives(self, now, prev, dt):
        return (np.array(now)-np.array(prev))/dt

    def getMaxOfMeans(self):
        return np.max(self.getMeanM1M2Values())

    def getM1(self):
        return self.m1

    def getM2(self):
        return self.m2

class DeckState(Enum):
    UNDEFINED = -1
    UP = 1
    DOWN = 0

class Node:
    def __init__(self):
        self.serial_lock = Lock()
        # rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        self.dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        self.baud_rate = int(rospy.get_param("~baud", "115200"))
        self.address = int(rospy.get_param("~address", "128"))

        # Parameter to indicate that the motors are running.
        self.is_running = False
        rospy.sleep(2)
        # Check port permissions
        if (self.port_permissions(self.dev_name)):
            rospy.loginfo("Port permissions are acceptable.")
        else:
            rospy.logwarn(
                "Port permissions are not valid. (sudo chmod 777 [port name])")
            rospy.logwarn("Try to give permissions...")
            if (not self.give_port_permissions(self.dev_name)):
                rospy.logerr("cannot achieved")
                exit(-1)
            rospy.loginfo("Permissions are now ok!")
        rospy.sleep(2)
        # Create a roboclaw instance
        self.roboclaw = Roboclaw(self.dev_name, self.baud_rate,timeout=0.02,retries=1)

        # Open roboclaw port and check version
        self.open_roboclaw_port()

        # Topics
        self.m1_current_last_pub = rospy.Publisher(
            'm1_current_last', FloatStamped, queue_size=10)
        self.m2_current_last_pub = rospy.Publisher(
            'm2_current_last', FloatStamped, queue_size=10)
        
        self.m1_current_pub = rospy.Publisher(
            'm1_current', FloatStamped, queue_size=10)
        self.m2_current_pub = rospy.Publisher(
            'm2_current', FloatStamped, queue_size=10)
        self.deck_position_pub = rospy.Publisher(
            'deck_position', String, queue_size=10, latch=True)
        self.status_pub = rospy.Publisher('status', String, queue_size=5)
        self.temp_pub = rospy.Publisher('temperature', FloatStamped, queue_size=5)
        self._pwms = rospy.Publisher('pwm', Float32MultiArray, queue_size=5)

        self.current_msg = FloatStamped()

        # Motor Currents Class
        self.motorCurrents = MotorCurrents()

        # Services
        self.deck_control_srv = rospy.Service(
            'move_prismatic', SetBool, self.deck_control_cb)
        
        # Action Server
        self.action_server = SimpleActionServer(
            'move_prismatic_action',
            MovePrismaticAction,
            execute_cb=self.execute_action_cb,
            auto_start=False
        )
        self.action_server.start()

        self.outputpower = 0.0
        # params
        self._power_stop_threshold = rospy.get_param("~power_stop_threshold", 0.3) #0.3
        self._stop_move_timeout = rospy.get_param("~stop_move_timeout", 20) #20
        self._pwm_duty_cicle = rospy.get_param("~pwm_duty_cycle", 65) #65
        self._max_cnt_to_stop = rospy.get_param("~max_counter_stop", 5) #5
        self._publish_roboclaw_temperature = rospy.get_param("~publish_temperature", True)
        self._publish_roboclaw_status = rospy.get_param("~publish_status", True)
        self._publish_currents = rospy.get_param("~publish_currents", True)
        self._stop_with_time = rospy.get_param("~stop_with_time",False)
        self._stop_with_time_seconds = rospy.get_param("~stop_with_time_seconds",10.0)
        self._inverted_logic = rospy.get_param("~service_inverted_logic",False)
        self._ce_block_enabled = rospy.get_param("mower/ce_block_enabled",False)
        self._emergency_enabled = rospy.get_param("mower/emergency_stop",False)
        rate = rospy.get_param("~rate",10.0)
        publish_rate = rospy.get_param("~publish_rate",5.0)
        self._deck_state = DeckState.UNDEFINED
        rospy.set_param("deck/state", DeckState.UNDEFINED.value)
        # Timers
        self.period = rospy.Duration().from_sec(1.0/publish_rate)
        self.timer_update_rate = rospy.Duration().from_sec(1.0/rate)
        self.timer1 = rospy.Timer(self.timer_update_rate, self._read_data_callback)
    
    def _publish_callback(self,timer):
        # Publish Mean Current Messages
        if (self._publish_currents and (self.m1_current_pub.get_num_connections()>0 or self.m2_current_pub.get_num_connections()>0)):
            mean_currents = self.motorCurrents.getMeanM1M2Values()
            msg = FloatStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = mean_currents[0]
            self.m1_current_pub.publish(msg)
            msg.data = mean_currents[1]
            self.m2_current_pub.publish(msg)
            
            # ! ADDED FOR DEBUGGING:
            # current_1_last = self.motorCurrents.getM1()
            # current_2_last = self.motorCurrents.getM2()
            # msg.data = current_1_last[-1]
            # self.m1_current_last_pub.publish(msg)
            # msg.data = current_2_last[-1]
            # self.m2_current_last_pub.publish(msg)
            # ! ADDED FOR DEBUGGING
        
        # Read and publish Errors
        self.serial_lock.acquire()
        if (self._publish_roboclaw_status and self.status_pub.get_num_connections()>0):
            res = self.read_list_of_errors()
            if (res!=None):
                self.publish_list_of_errors(res)
        
        # Read and publish Temp
        if (self._publish_roboclaw_temperature and self.temp_pub.get_num_connections()>0):
            res = self.read_temps()
            if (res!=None):
                self.publish_temperature(res/10)
        
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
                

    def _read_data_callback(self, timer):
        # Read and append currents to the current instance
        #tick = rospy.Time.now().to_sec()
        self.serial_lock.acquire()
        res = self.read_currents()
        if(self.serial_lock.locked()):
            self.serial_lock.release()
        if (res!=None):
            m1_current, m2_current = res
            #print("Time to read Currents: {}".format(rospy.Time.now().to_sec()-tick))
            self.motorCurrents.appendM1(m1_current)
            self.motorCurrents.appendM2(m2_current)
        
            # get mean of motors current values
        
            mean_currents = self.motorCurrents.getMeanM1M2Values()
        
            # Calculate Output power
            sum_of_means = (mean_currents[0]*mean_currents[0])+(mean_currents[1]*mean_currents[1])
            self.outputpower = sum_of_means/10000.0
        
    def open_roboclaw_port(self):
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
        else:
            rospy.logdebug(repr(version[1]))

    def port_permissions(self, port):
        try:
            import os
            if (os.access(port, os.R_OK) and os.access(port, os.W_OK) and os.access(port, os.X_OK)):
                return True
            else:
                return False
        except:
            rospy.logerror('Roboclaw: Cannot read port permissions')

    def give_port_permissions(self, port):
        import os
        if os.path.exists(port):
            os.system('echo INnovation! | sudo -S chmod 777 '+port)
            return True
        else:
            return False

    def publish_list_of_errors(self, lst):
        msg = String()
        msg.data = str(lst)
        self.status_pub.publish(msg)
    
    def publish_temperature(self, temp):
        msg = FloatStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = temp
        self.temp_pub.publish(msg)

    def run(self):
        rospy.loginfo("Starting motor drive")
        import time
        time.sleep(1)
        timer2 = rospy.Timer(self.period, self._publish_callback)
        while(not rospy.is_shutdown()):
            rospy.sleep(1)

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

    def deck_control_cb(self, req):
        try:
            # Invert logic if necessary
            if self._inverted_logic:
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

    def send_zero_commands(self):
        self.serial_lock.acquire()
        self.is_running = False
        self.roboclaw.ForwardM1(self.address, 0)
        self.roboclaw.ForwardM2(self.address, 0)
        rospy.sleep(0.1)
        self.roboclaw.ForwardM1(self.address, 0)
        self.roboclaw.ForwardM2(self.address, 0)
        rospy.sleep(0.1)
        self.roboclaw.ForwardM1(self.address, 0)
        self.roboclaw.ForwardM2(self.address, 0)
        if (self.serial_lock.locked()):
            self.serial_lock.release()


    def update_deck_state(self, cmd):
        rospy.loginfo("Updating deck state to: %s" % cmd.name) 
        msg = String()
        if cmd == DeckState.UNDEFINED:
            self._deck_state = DeckState.UNDEFINED
            rospy.set_param("deck/state", DeckState.UNDEFINED.value)
            msg.data = cmd.name
        elif cmd == DeckState.DOWN:
            self._deck_state = DeckState.DOWN
            rospy.set_param("deck/state",  DeckState.DOWN.value)
            msg.data = cmd.name
        elif cmd == DeckState.UP:
            self._deck_state = DeckState.UP
            rospy.set_param("deck/state", DeckState.UP.value)
            msg.data = cmd.name
        self.deck_position_pub.publish(msg)

    def execute_action_cb(self, goal):
        feedback = MovePrismaticFeedback()
        result = MovePrismaticResult()

        try:        
            # Call roboclaw_control with the goal position
            if self._inverted_logic:
                goal.position = not goal.position
                
            cmd = DeckState.UP if goal.position else DeckState.DOWN
            time_started = rospy.Time.now().to_sec()
            cnt = 0

            if cmd == self._deck_state:
                rospy.logwarn("Deck already in this position. Not moving.")
                result.success = True
                result.message = "Deck already in this position."
                self.action_server.set_succeeded(result)
                return

            rospy.logwarn("- Deck goes to %s position" % ("lower" if cmd == DeckState.DOWN else "upper"))
            self.is_running = True

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
                    self.send_zero_commands()  # Stop the motors
                    # Update the deck state to undefined
                    self.update_deck_state(DeckState.UNDEFINED)
                    result.success = False
                    result.message = "Operation preempted by another goal."
                    self.action_server.set_preempted(result)
                    self.is_running = False
                    return

                duration = rospy.Time.now().to_sec() - time_started

                # Continuously send motor commands
                self.serial_lock.acquire()
                if cmd == DeckState.UP:
                    self.roboclaw.BackwardM1(self.address, int(self._pwm_duty_cicle * 1.055))
                    self.roboclaw.BackwardM2(self.address, self._pwm_duty_cicle)
                else:
                    self.roboclaw.ForwardM1(self.address, int(self._pwm_duty_cicle * 1.055))
                    self.roboclaw.ForwardM2(self.address, self._pwm_duty_cicle)
                if self.serial_lock.locked():
                    self.serial_lock.release()

                # Publish feedback
                feedback.power_output = self.outputpower
                feedback.iterations = cnt
                feedback.duration = duration
                self.action_server.publish_feedback(feedback)

                # Timeout
                if duration > self._stop_move_timeout:
                    self.send_zero_commands()
                    result.success = False
                    result.message = "Timeout reached!"
                    self.action_server.set_aborted(result)
                    return

                # Stop with time
                elif self._stop_with_time and duration >= self._stop_with_time_seconds:
                    rospy.sleep(1)
                    self.send_zero_commands()
                    self.update_deck_state(cmd)
                    result.success = True
                    result.message = "Position reached."
                    self.action_server.set_succeeded(result)
                    return

                # Stop based on power threshold
                elif duration > 2 and self.outputpower < self._power_stop_threshold and not self._stop_with_time:
                    cnt += 1
                    if cnt > self._max_cnt_to_stop:
                        rospy.sleep(1)
                        self.send_zero_commands()
                        self.update_deck_state(cmd)
                        result.success = True
                        result.message = "Position reached."
                        self.action_server.set_succeeded(result)
                        return

                rospy.sleep(self.timer_update_rate.to_sec())

            # If shutdown is triggered
            self.send_zero_commands()
            result.success = False
            result.message = "Operation interrupted by shutdown."
            self.action_server.set_aborted(result)

        except Exception as e:
            rospy.logerr("Unexpected error in execute_action_cb: %s" % str(e))
            self.send_zero_commands()
            result.success = False
            result.message = "Unexpected error: %s" % str(e)
            self.action_server.set_aborted(result)

if __name__ == "__main__":
    try:
        rospy.init_node("roboclaw_node")
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
