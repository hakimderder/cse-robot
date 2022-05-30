#!/usr/bin/python

"""
Class tests
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import time



class Gnss_testing():
    def __init__(self, param1):
        rospy.loginfo("Setting Up the Node...")
        rospy.init_node('xsens_mti_node')
        rospy.loginfo("> xsens_mti_node initialized")
        self.params = { "param1": param1 }
        rospy.loginfo("> parameters corrrectly saved")
        # subscription
        rospy.Subsciber("gnss", )
        self.datas = set()
        rospy.loginfo("Initialization complete")

    def run(self):
        #--- Set the control rate
        rate = rospy.Rate(20)
        rospy.loginfo("speed multiplier = %f"%(speed_limit_value))
        rospy.loginfo("step value = %d"%(up_down_step))
        rospy.loginfo("max PWM = %d"%(vehicle_max_speed))
        while not rospy.is_shutdown():
            rate.sleep()
            #self.send_servo_msg()
        GPIO.cleanup()

    def callback(data):
        self.datas.add(data.data)
        rospy.loginfo("recieved this data : {}".format(data.data))


if __name__ == "__main__":
    dk_llc     = DkLowLevelCtrl()
    dk_llc.run()
