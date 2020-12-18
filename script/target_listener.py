#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    float_converted_tuple = tuple(float(num) for num in data.data.replace('(', '').replace(')', '').split(', ')) 
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo(float_converted_tuple)
    x,y = float_converted_tuple
    rospy.loginfo("The x value of the tuple is %4.2f and the y component is %6.3f", x, y)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("target", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()