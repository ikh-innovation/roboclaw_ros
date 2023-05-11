#!/usr/bin/env python
import rospy
from ikh_ros_msgs.msg import FloatStamped
from std_srvs.srv import SetBool
from std_msgs.msg import String
import numpy as np
from roboclaw_ros.roboclaw_driver import Roboclaw


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


class Node:
    def __init__(self):
        rospy.init_node("roboclaw_node")
        # rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        self.dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        self.baud_rate = int(rospy.get_param("~baud", "115200"))
        self.address = int(rospy.get_param("~address", "128"))

        # Parameter to indicate that the motors are running.
        self.is_running = False

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

        # Create a roboclaw instance
        self.roboclaw = Roboclaw(self.dev_name, self.baud_rate)

        # Open roboclaw port and check version
        self.open_roboclaw_port()

        # Topics
        self.m1_current_pub = rospy.Publisher(
            'm1_current', FloatStamped, queue_size=10)
        self.m2_current_pub = rospy.Publisher(
            'm2_current', FloatStamped, queue_size=10)
        self.deck_position_pub = rospy.Publisher(
            'deck_position', String, queue_size=10, latch=True)
        self.status_pub = rospy.Publisher('status', String, queue_size=10)
        self.temp_pub = rospy.Publisher('temperature', FloatStamped, queue_size=10)

        self.current_msg = FloatStamped()

        # Motor Currents Class
        self.motorCurrents = MotorCurrents()

        # Services
        self.deck_control_srv = rospy.Service(
            'move_prismatic', SetBool, self.deck_control_cb)

        self.outputpower = 0.0
        # params
        self._power_stop_threshold = rospy.get_param("~power_stop_threshold", 0.3) #0.3
        self._stop_move_timeout = rospy.get_param("~stop_move_timeout", 20) #20
        self._pwm_duty_cicle = rospy.get_param("~pwm_duty_cycle", 65) #65
        self._max_cnt_to_stop = rospy.get_param("~max_counter_stop", 5) #5
        self._publish_roboclaw_temperature = rospy.get_param("~publish_temperature", True)
        self._publish_roboclaw_status = rospy.get_param("~publish_status", True)
        self._publish_currents = rospy.get_param("~publish_currents", True)
        rate = rospy.get_param("~rate",10.0)
        publish_rate = rospy.get_param("~publish_rate",5.0)
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
        
        # Read and publish Errors
        if (self._publish_roboclaw_status and self.status_pub.get_num_connections()>0):
            self.publish_list_of_errors(self.read_list_of_errors())
        
        # Read and publish Temp
        if (self._publish_roboclaw_temperature and self.temp_pub.get_num_connections()>0):
            self.publish_temperature(self.read_temps()/10)
                

    def _read_data_callback(self, timer):
        # Read and append currents to the current instance
        m1_current, m2_current = self.read_currents()
        self.motorCurrents.appendM1(m1_current)
        self.motorCurrents.appendM2(m2_current)
        
        # get mean of motors current values
        mean_currents = self.motorCurrents.getMeanM1M2Values()
        
        # Calculate Output power
        sum_of_means = mean_currents[0]+mean_currents[1]
        sum_of_means /= 100
        self.outputpower = sum_of_means*sum_of_means
        
        

        

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
            
            return self.roboclaw.ReadErrorDecoded(self.address)
        except:
            raise Exception("Cannot read roboclaw errors")

    def read_temps(self):
        try:
            temp1, temp2 = self.roboclaw.ReadTemp(self.address)
            return temp2
        except:
            raise Exception("Cannot read roboclaw errors")
    
    def read_currents(self):
        try:
            _none, m1_current, m2_current = self.roboclaw.ReadCurrents(
            self.address)
            return (m1_current, m2_current)
        except:
            raise Exception("Cannot read roboclaw motor currents")

    def deck_control_cb(self, req):
        res = (False, "Nothing")
        if req.data:
            res = self.roboclaw_control(1)
        else:
            res = self.roboclaw_control(0)
        return res

    def send_zero_commands(self):
        self.is_running = False
        self.roboclaw.ForwardM1(self.address, 0)
        self.roboclaw.ForwardM2(self.address, 0)

    def update_deck_state(self, cmd):
        msg = String()
        if (cmd):
            msg.data = "down"
            self.deck_position_pub.publish(msg)
        else:
            msg.data = "up"
            self.deck_position_pub.publish(msg)

    def roboclaw_control(self, cmd):
        time_stared = rospy.Time.now().to_sec()
        if (not self.is_running):
            if (cmd):
                rospy.logwarn("- Deck goes to lower position")
                self.is_running = True
                self.roboclaw.BackwardM1(
                    self.address, int(self._pwm_duty_cicle*1.055))
                self.roboclaw.BackwardM2(self.address, self._pwm_duty_cicle)
            else:
                rospy.logwarn("- Deck goes to upper position")
                self.is_running = True
                self.roboclaw.ForwardM1(
                    self.address, int(self._pwm_duty_cicle*1.055))
                self.roboclaw.ForwardM2(self.address, self._pwm_duty_cicle)
        else:
            return (False, "An other command is ongoing! Try later.")

        cnt = 0

        while (not rospy.is_shutdown()):

            duration = rospy.Time.now().to_sec()-time_stared
            
            # if timeout excited then stop motors and return false with the related message
            if (duration > self._stop_move_timeout):
                self.send_zero_commands()
                return (False, "Timeout reached!")
            
            # if duration > 2 secs and the total output power is lower than threshold the motor stops
            elif ((duration > 2) and (self.outputpower < self._power_stop_threshold)):
                cnt += 1
                if (cnt > self._max_cnt_to_stop):
                    rospy.sleep(1)
                    # stop motors
                    self.send_zero_commands()
                    # update state
                    self.update_deck_state(cmd)
                    return (True, "Position Reached")
                
            rospy.sleep(self.timer_update_rate.to_sec())


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
