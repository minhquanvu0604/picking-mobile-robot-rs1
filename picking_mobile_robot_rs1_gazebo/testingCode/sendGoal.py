#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#NEED TO ADD PUBLISHER TO CONSUMER TOPIC (/BATTERY/CONSUMER/0) AT 9 AMPS WHILE ROBOT IS MOVING. 
#NEED TO ADD SERVICE CALL TO BATTERY CHARGE TO TOP UP BATTERY WHEN AT HOME STATION
#NEED TO ADD BATTERY CHECKER BEFORE MOVING TO RETURN TO HOME BEFORE MOVING. 
class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher_node', anonymous=True)

        # Using ros action
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.client.wait_for_server()

        self.list_index = None
    
    # question?????????
    def bubble_sort(self, currentGoalList):
        n = len(currentGoalList)
        for i in range(n):
            # Flag to optimize the sorting process
            swapped = False

            # Last i elements are already in place, so we don't need to check them
            for j in range(0, n-i-1):
                # Swap if the element found is greater than the next element
                if currentGoalList[j] > currentGoalList[j+1]:
                    currentGoalList[j], currentGoalList[j+1] = currentGoalList[j+1], currentGoalList[j]
                    swapped = True

            # If no two elements were swapped in inner loop, the array is already sorted
            if not swapped:
                break
    

    def generate_pose(self, goal):

        if goal > 0 and goal < 5:
            goal_x = 6 - 2*(goal - 1)
            goal_y = 1

        elif goal > 4 and goal < 9:
            goal_x = 6 - 2*(goal - 5)
            goal_y = -2

        elif goal > 8 and goal < 13:
            goal_x = 6 - 2*(goal - 8)
            goal_y = -5

        elif goal > 12 and goal < 19:
            goal_x = 6 - 2*(goal - 13)
            goal_y = -8

        elif goal > 18 and goal < 25:
            goal_x = -5
            goal_y = -8 + 2*(goal - 19)

        else:
            goal_x = -5
            goal_y = 2
        
        goalPose = []
        goalPose.append(goal_x, goal_y)
        return goalPose
    

    def run_mission(self):

        goal_msg = MoveBaseGoal()

        goal_msg.target_pose.header.frame_id = 'map'

        seq += 1

        goal_msg.target_pose.header.stamp = rospy.Time.now()

        for i in (self.list_index):
            goalPose = self.generate_pose(i)

            goal_x = goalPose[0]
            goal_y = goalPose[1]

            goal_msg.target_pose.pose.position.x = goal_x
            goal_msg.target_pose.pose.position.y = goal_y
            goal_msg.target_pose.pose.position.z = 0
            goal_msg.target_pose.pose.orientation.x = 0
            goal_msg.target_pose.pose.orientation.y = 0
            goal_msg.target_pose.pose.orientation.z = 0
            goal_msg.target_pose.pose.orientation.w = 1

            # Publish the goal message
            self.client.send_goal(goal_msg)

            # Wait for result, turtlebot 
            self.client.wait_for_result()

        rospy.sleep(1)


if __name__ == '__main__':
    try:
        goal_publisher = GoalPublisher()
        goal_publisher.publish_goal()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("FAIL")
        pass
