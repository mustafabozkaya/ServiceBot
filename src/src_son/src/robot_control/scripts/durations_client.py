#!/usr/bin/env python3
import rospkg
import rospy
from robot_control.srv import Durations,DurationsRequest

def call_service(goal):
    try:
        rospy.init_node('service_client') # Initialise a ROS node with the name service_client
        rospy.wait_for_service('/duration_service') # Wait for the service client /move_bb8_in_circle_custom to be running
        service_client = rospy.ServiceProxy('/duration_service', Durations) # Create the connection to the service
        request_object = DurationsRequest() # Create an object of type EmptyRequest

        
        request_object.target = goal

        rospy.loginfo("Doing Service Call...")
        result = service_client(request_object) # Send through the connection the path to the trajectory file to be executed
        rospy.loginfo("response result :"+str(result)) # Print the result given by the service called

        rospy.loginfo("END of Service call...")


    except rospy.ServiceException as e:
        print("service not called ")
        print(e)

target=float(input("type move target (m): "))
call_service(target)




