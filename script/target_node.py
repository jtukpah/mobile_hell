#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def target():
    pub = rospy.Publisher('target', String, queue_size=10)
    rospy.init_node('mazemover', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goal = "(1.78, -0.274)"
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        target()
    except rospy.ROSInterruptException:
        pass