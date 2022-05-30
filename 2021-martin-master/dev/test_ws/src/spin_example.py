# source : https://stackoverflow.com/questions/68138490/how-to-create-a-ros-node-that-can-start-and-stop-counting-based-on-what-message
#!/usr/bin/env python

#imports
import rospy
from std_msgs.msg import String, Int32

class NodeClass:
    def __init__(self):
        #initialize node
        rospy.init_node("count_node")
        self.nodename = rospy.get_name()
        rospy.loginfo("Started node %s" % self.nodename)
        
        #vars
        self.percent = 0

        #params and ROS params
        self.rate = rospy.get_param('~rate',10)

        #subscribers
        rospy.Subscriber("brwsrButtons", String, self.callback)
        #publishers
        self.progressPub = rospy.Publisher("progress", Int32, queue_size=10)

    def callback(self, msg):
        if msg.data == 'start_count' and self.percent<100:
            self.percent += 1
        else:
            self.percent = 0

    def spin(self):
        self.r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            percentPubMsg = Int32()
            percentPubMsg.data = self.percent
            self.progressPub.publish(percentPubMsg)
            self.r.sleep()

if __name__ == '__main__':
    """main"""
    try:
        nodeClass = NodeClass()
        nodeClass.spin()
    except rospy.ROSInterruptException:
        pass
