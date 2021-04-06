#! /usr/bin/env python

import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

"""
# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    """
    Error that might jump
    
    self._feedback.lastImage = 
AttributeError: 'ArdroneAS' obj
    
    """
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

# initializes the action client node
rospy.init_node('exercise_action_client')

action_server_name = '/ardrone_action_server'
client = actionlib.SimpleActionClient(action_server_name, ArdroneAction)

# creates publishers
pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
pub_take_off = rospy.Publisher("/drone/takeoff", Empty, queue_size=1)
pub_land = rospy.Publisher("/drone/land", Empty, queue_size=1)

# creates messages to publish
move = Twist()
takeoff = Empty()
land = Empty()

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

# creates a goal to send to the action server
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

client.send_goal(goal, feedback_cb=feedback_callback)


# You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
#Its a variable, better use a function like get_state.
#state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
state_result = client.get_state()

rate = rospy.Rate(1)

rospy.loginfo("state_result: "+str(state_result))

#We takeoff the drone during the first 3 seconds
c = 0
while not c == 3:
    pub_take_off.publish(takeoff)
    rospy.loginfo('Taking off!')
    time.sleep(1)
    c += 1

#We move the drone in a circle wile the state of the Action is not DONE yet
while state_result < DONE:
    rospy.loginfo("The quadcopter will now move around")
    move.linear.x = 1
    move.angular.z = 1
    pub_cmd_vel.publish(move)
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    
rospy.loginfo("[Result] State: "+str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")

# We land the drone when the action is finished
c = 0
while not c == 3:
    move.linear.x = 0
    move.angular.z = 0
    pub_cmd_vel.publish(move)
    pub_land.publish(land)
    rospy.loginfo('Landing...')
    time.sleep(1)
    c += 1

#rospy.loginfo("[Result] State: "+str(client.get_result()))