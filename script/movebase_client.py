#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

tuple_x_value = 0.0
tuple_y_value = 0.0

def callback(data):
    float_converted_tuple = tuple(float(num) for num in data.data.replace('(', '').replace(')', '').split(', ')) 
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo(float_converted_tuple)
    tuple_x_value,tuple_y_value = float_converted_tuple
    rospy.loginfo("The penguin x value of the tuple is %4.2f and the y component is %6.3f", tuple_x_value, tuple_y_value)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("target", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def movebase_client(tuple_x_value,tuple_y_value):

    #rospy.Subscriber("target", String, callback)

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = tuple_x_value
    goal.target_pose.pose.position.y = tuple_y_value
    goal.target_pose.pose.position.y = -0.00143
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        listener()
        rospy.loginfo("Puppy x value of the tuple is %4.2f and the y component is %6.3f", tuple_x_value, tuple_y_value)
        result = movebase_client(tuple_x_value, tuple_y_value)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
