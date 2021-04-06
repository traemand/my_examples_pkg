#! /usr/bin/env python

import rospy
from trajectory_by_name_srv.srv import TrajByName, TrajByNameRequest    # Import the service message used by the service /trajectory_by_name
import sys


rospy.init_node('service_client')   # Initialise a ROS node with the name service_client

rospy.wait_for_service('/trajectory_by_name')   # Wait for the service client /trajectory_by_name to be running

traj_by_name_service = rospy.ServiceProxy('/trajectory_by_name', TrajByName)    # Create the connection to the service

traj_by_name_object = TrajByNameRequest()   # Create an object of type TrajByNameRequest

traj_by_name_object.traj_name = "release_food"  # Fill the variable traj_name of this object with the desired value

result = traj_by_name_service(traj_by_name_object)  # Send through the connection the name of the trajectory to be executed by the robot

print(result)   # Print the result given by the service called
