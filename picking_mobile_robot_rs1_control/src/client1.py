#!/usr/bin/env python3

# read packing_list.csv to get list of products, empty upon success
# query into a deque
# publish one by one, move on upon successful reach
# 
#

import rospy
import csv
import rospkg
import actionlib

from std_srvs.srv import Trigger, TriggerResponse
from collections import deque
from std_msgs.msg import String
from std_msgs.msg import Float32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_ros_battery.srv import Reset

#NEED TO ADD PUBLISHER TO CONSUMER TOPIC (/BATTERY/CONSUMER/0) AT 9 AMPS WHILE ROBOT IS MOVING. 
#NEED TO ADD SERVICE CALL TO BATTERY CHARGE TO TOP UP BATTERY WHEN AT HOME STATION
#NEED TO ADD BATTERY CHECKER BEFORE MOVING TO RETURN TO HOME BEFORE MOVING. 
class CsvReaderNode:
    def __init__(self):

        # Define the queue of the number from 1-26
        self.queue = deque()      

        # Initialize the ROS node
        rospy.init_node('mission_handler')

        # Voltage publisher (change it to service)
        rospy.wait_for_service('/battery/reset')
        self.charger = rospy.ServiceProxy('/battery/reset', Reset)
        
        # Get package path
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('picking_mobile_robot_rs1_control')
        
        # Define the file path relative to the package
        self.packing_list_path = package_path + '/csv/packing_list.csv'
        self.product_location_path = package_path + '/csv/product_location.csv'

        # Create a service server
        self.service = rospy.Service('read_csv', Trigger, self.csv_callback)

        # Create an action client to communicate with move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()

        self.consumer0 = rospy.Publisher('battery/consumer/0', Float32, queue_size=5)
        self.consumer1 = rospy.Publisher('battery/consumer/1', Float32, queue_size= 5)

        rospy.loginfo("Connected to move_base action server")


    # change to service call
    def reset_voltage(self):
        
        return None

    def battery_state_check(self):
        voltage = rospy.wait_for_message('/battery/voltage', Float32)
        
        if voltage.data < 9.5:
            rospy.loginfo("LOW BATTERY, GOING BACK TO CHARGE STATION")

            self.return_home()

            rospy.loginfo("CHARGING")
            self.charger(True)
            rospy.sleep(10)
            # reset the voltage back to normal condition



    def csv_callback(self, req):
        response = TriggerResponse()
        try:
            # Read the CSV and append its rows to the deque
            with open(self.packing_list_path, 'r') as csv_file:
                reader = csv.reader(csv_file)
                for row in reader:
                    product_number = self.get_number_from_name(row[0])
                    self.queue.append(product_number)
            # Clear the contents of the CSV
            with open(self.packing_list_path, 'w') as csv_file:
                pass  # Do nothing, effectively emptying the file

            # Sort the queue  
            self.bubble_sort(self.queue)

            response.success = True
            response.message = "CSV file read successfully and data added to the queue."
            # self.queue_publisher.publish(self.queue)
        except Exception as e:
            response.success = False
            response.message = "Failed to read the CSV file. Error: " + str(e)

            import traceback
            print(traceback.format_exc())
            return response
        
        return response
    
    def moveRobot(self, goal_msg):
        # Publish the goal message
        self.move_base_client.send_goal(goal_msg)
        moveConsume = Float32()
        moveConsume.date = 4.5
        self.consumer1.publish(moveConsume)
        
        # Wait for result, turtlebot 
        self.move_base_client.wait_for_result()
        moveConsume.date = 0
        self.consumer1.publish(moveConsume)

    
    def deposit_zone(self):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        goal_msg.target_pose.pose.position.x = -6
        goal_msg.target_pose.pose.position.y = 10
        goal_msg.target_pose.pose.position.z = 0
        goal_msg.target_pose.pose.orientation.x = 0
        goal_msg.target_pose.pose.orientation.y = 0
        goal_msg.target_pose.pose.orientation.z = 0
        goal_msg.target_pose.pose.orientation.w = 1

        rospy.loginfo("DEPOSITING ORDER")

        self.moveRobot(goal_msg)

        rospy.loginfo("DEPOSITED")

    def return_home(self):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        goal_msg.target_pose.pose.position.x = -6
        goal_msg.target_pose.pose.position.y = 10
        goal_msg.target_pose.pose.position.z = 0
        goal_msg.target_pose.pose.orientation.x = 0
        goal_msg.target_pose.pose.orientation.y = 0
        goal_msg.target_pose.pose.orientation.z = 0
        goal_msg.target_pose.pose.orientation.w = 1


        rospy.loginfo("RETURNING HOME")

        self.moveRobot(goal_msg)

        rospy.loginfo("FINISHED")

    # Get the goal pose from a number between 1-26 
    def get_pose(self, goal):
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
        goalPose.append(goal_x)
        goalPose.append(goal_y)
        return goalPose
    

    def get_number_from_name(self, fruit_name):
        with open(self.product_location_path, 'r') as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                if row['fruit_name'] == fruit_name:
                    return int(row['quantity'])
        rospy.logwarn('This product is not available in the product list.')
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

    def send_next_goal(self):
        if self.queue:
            # Check the battery firt
            # self.battery_state_check()

            next_goal_number = self.queue.popleft()
            goal_pose = self.get_pose(next_goal_number)

            goal_msg = MoveBaseGoal()
            goal_msg.target_pose.header.frame_id = 'map'
            goal_msg.target_pose.header.stamp = rospy.Time.now()

            goal_msg.target_pose.pose.position.x = goal_pose[0]
            goal_msg.target_pose.pose.position.y = goal_pose[1]
            goal_msg.target_pose.pose.position.z = 0
            goal_msg.target_pose.pose.orientation.x = 0
            goal_msg.target_pose.pose.orientation.y = 0
            goal_msg.target_pose.pose.orientation.z = 0
            goal_msg.target_pose.pose.orientation.w = 1


            rospy.loginfo("SENDING NEXT GOAL")

            self.moveRobot(goal_msg)

            # Wait for placing product on turtlebot
            rospy.loginfo("WAITING FOR PICKING")
            rospy.sleep(5.0)




if __name__ == "__main__":
    node = CsvReaderNode()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        if (node.queue):
            node.send_next_goal()
        else:
            node.return_home()
        baseConsumption = Float32
        baseConsumption.data = 1.5 
        node.consumer0.publish(baseConsumption)
        rate.sleep()
