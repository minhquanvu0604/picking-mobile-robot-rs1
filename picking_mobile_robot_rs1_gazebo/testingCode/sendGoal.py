#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher_node', anonymous=True)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        self.result_subcriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)

        self.result = None

    def result_callback(self, msg):
        # Access and print the result field of the MoveBaseActionResult message
        self.result = msg.status.text
        print("Received result:", self.result)
    
    
    def letter_position(self, value):
        # Convert the value to lowercase to handle both uppercase and lowercase letters
        value = value.lower()
        
        # Check if the value is a single alphabetical character
        if len(value) == 1 and value.isalpha():
            # Calculate the position in the alphabet (1-based)
            position = ord(value) - ord('a') + 1
            return position
        
        # If the value is not a single alphabetical character, return None
        return None
    

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
    
    def generate_goal(self):

        goalList = []

        # Get number of product
        num = int(input("Enter number of product: "))

        for i in num:
            # Get product
            product = input("Enter the section: ")

            # Get first letter of product
            first_letter = product[0]
            
            # Get the goal correspond to first letter of product
            goal = self.letter_position(first_letter)

            goalList.append(goal)

        newGoalList = self.bubble_sort(goalList)

        return newGoalList
    

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
    

    def publish_goal(self):
        goalList = self.generate_goal()

        # this is only for idea //////////////////////////////
        for goal in goalList:
            goalPose = self.generate_pose(goal)
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = goalPose[0]
            goal_msg.pose.position.y = goalPose[1]
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.x = 0.0
            goal_msg.pose.orientation.y = 0.0
            goal_msg.pose.orientation.z = 0.0
            goal_msg.pose.orientation.w = 1.0
            
            # Set the frame ID (e.g., 'map' or any relevant frame)
            goal_msg.header.frame_id = 'map'

            # Set the timestamp
            # goal_msg.header.stamp = rospy.Time.now()

            # Publish the goal message
            self.goal_publisher.publish(goal_msg)
            while True:
                if self.result == "Goal reached.":
                    break
            rospy.sleep(1)
        # //////////////////////////////////////
        
        # Set the position and orientation values (you can modify these as needed)
        goal_msg.pose.position.x = -6.0
        goal_msg.pose.position.y = 10

        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        goal_msg.header.frame_id = 'map'

        # Publish the goal message
        self.goal_publisher.publish(goal_msg)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        goal_publisher = GoalPublisher()
        goal_publisher.publish_goal()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("FAIL")
        pass

