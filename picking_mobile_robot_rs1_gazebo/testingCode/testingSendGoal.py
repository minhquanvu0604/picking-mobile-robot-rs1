#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():
    # Initialize the ROS node with a unique name
    rospy.init_node('goal_publisher_node', anonymous=True)

    # Create a publisher for the '/move_base_simple/goal' topic
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    r = rospy.Rate(0.1) # 10hz

    while(True):
	    # Create a PoseStamped message
	    goal_msg = PoseStamped()
	    # Create data input from user
	    # try:
	    #     # Get user input for the goal position (x and y coordinates)
	    #     section = float(input("Enter the section: "))
	    # except ValueError:
	    #     rospy.logerr("Invalid input. Please enter valid numerical values for X and Y.")
	    #     return
	    # tu.msg.PoseStamped.pose.position
	    # Set the position and orientation values (you can modify these as needed)
	    goal_msg.pose.position.x = 1.0
	    goal_msg.pose.position.y = 1.0

	    goal_msg.pose.position.z = 0.0
	    goal_msg.pose.orientation.x = 0.0
	    goal_msg.pose.orientation.y = 0.0
	    goal_msg.pose.orientation.z = 0.0
	    goal_msg.pose.orientation.w = 1.0

	    # Set the frame ID (e.g., 'map' or any relevant frame)
	    goal_msg.header.frame_id = 'map'

	    # Set the timestamp
	    # goal_msg.header.stamp = rospy.Time.now()

	    goal_msg.header.seq = 3


	    # Publish the goal message
	    goal_publisher.publish(goal_msg)
	    
	    r.sleep()
	    

    # Spin to allow the node to publish messages
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        print("FAIL")
        pass
