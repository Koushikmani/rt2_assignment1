#! /usr/bin/env python

"""
.. module:: go_to_point
    :platform: Unix
    :synopsis: Node implementing the go_to_point behavior
.. moduleauthor::Koushikmani Maskalmatti Lakshman
Publishes to:
    /cmd_vel (geometry_msgs.msg.Twist)
ServiceServer:
    /set_vel (rt2_assignment1.srv.SetVel)
    requested by the jupyter notebook
ActionServer:
    /go_to_point (rt2_assignment1.action.PoseAction)
    called by :mod:`state_machine`
            
Description:
This node controls the go_to_point conduct of
the non-holonomic robot through an action server.
A FSM is utilized to demonstrate the conduct at whatever point
another objective posture is gotten:
     line up with the objective position
     go directly to the objective position
     line up with the objective direction
     objective posoe came to

The maximum qualities for both straight and precise
speed are refreshed each time a solicitation for the
/set_vel administration is gotten.            
            
"""


import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
## publisher
pub_ = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
## Maximum angular speed
ub_a = 0.6
lb_a = -0.5
## Maximum linear speed
ub_d = 0.6

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    """
    Arrange the robot in an ideal manner
    
    The capacity is utilized either to arrange
    the robot toward the objective goal (x,y) position
    or then again, whenever it's reached, to accomplish the
    objective direction. It likewise changes to
    another state, contingent upon the current
    one (either introductory heading or last
    direction).
   
   Args:
        des_yaw (float): wanted yaw
        next_state (int): next state to set

    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)



def go_straight_ahead(des_pos):
 """
    Drive toward the goal
    Set the linear and angular speed
    depending on the distance to the 
    goal pose.
    Args:
        des_pos (Point):  desired (x, y) position
        
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
"""
    Stop the robot
    Set the robot linear and angular 
    velocity to 0.
    
    """

    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    
def go_to_point(req):
"""
    State machine implementation
    Set an appropriate behaviour depending
    on the current robot state, in orderd
    to reach the goal.
    The state machine keeps running until
    the goal is reached or the action is
    preempted (the goal gets cancelled).
    Args:
        goal (PoseActionGoal): (x,y,theta) goal pose
        
    """
    desired_position = Point()
    desired_position.x = req.x
    desired_position.y = req.y
    des_yaw = req.theta
    change_state(0)
    while True:
    	if state_ == 0:
    		fix_yaw(desired_position)
    	elif state_ == 1:
    		go_straight_ahead(desired_position)
    	elif state_ == 2:
    		fix_final_yaw(des_yaw)
    	elif state_ == 3:
    		done()
    		break
    return True

def main():
    global pub_
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    service = rospy.Service('/go_to_point', Position, go_to_point)
    rospy.spin()

if __name__ == '__main__':
    main()
