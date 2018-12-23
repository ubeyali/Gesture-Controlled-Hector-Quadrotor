#!/usr/bin/env python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from hector_uav_msgs.srv import EnableMotors
import sys, select, termios, tty
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Odometry

from math import isnan
import rospy
import time
import sys
import roslib
roslib.load_manifest('hector_quadrotor_smart_drone')
import rospy
import actionlib
from hector_quadrotor_smart_drone.msg import DroneAction
global motor_command_publisher
global obstacaleBool
global position
global direction
global hover_limit
global hover_check
hover_check=True
direction ='z'

class_indices =  {0: 'down', #down
                  1: 'left', #left_forward
                  2: 'one',  #backward    
                  3: 'open', #hover
                  4: 'right',#right_forward
                  5: 'two',  #forward   
                  6: 'up'}   #up
moveBindings = {
       	'u':(0, 0, 1, 0),	#up
        'd':(0, 0,-1, 0),	#down
	    'l':(1, 0, 0, 0.3),	#left_forward
        'r':(1, 0, 0,-0.3),	#right_forward
        'y':(0, 0, 0,1),	#right
        'f':( 1, 0, 0, 0),	#forward
	    'b':(-1, 0, 0, 0),	#backward
	    'z':( 0, 0, 0, 0),  #zero
 }

def hector_hover():
    global position
    global direction
    global hover_limit
    if position.z < hover_limit:
		direction='u'
    else:
		direction='d'

class DroneServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('Drone', DroneAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    global direction
    global hover_check
    global hover_limit

    if goal.gesture_id==0:
        direction='d'
    elif goal.gesture_id==1:
		direction='l'
    elif goal.gesture_id==2:
        direction='b'
    elif goal.gesture_id==3 and hover_check:
         hover_limit=position.z
         hover_check=False
    elif goal.gesture_id==4:
		direction ='r'
    elif goal.gesture_id==5:
        direction ='f'
    elif goal.gesture_id==6:
		direction ='u'


    if goal.gesture_id != 3:
        hover_check=True
    print('Gesture: ',class_indices[goal.gesture_id])
    self.server.set_succeeded()
   


def enableMotors():
    SERVICE_ENABLE_MOTORS = 'enable_motors'
    print ("Waiting for service enable_motors")
    rospy.wait_for_service(SERVICE_ENABLE_MOTORS)
    try:
        enable_motors = rospy.ServiceProxy(SERVICE_ENABLE_MOTORS, EnableMotors)
        res = enable_motors(True)
        if res:
            print ("Motors enabled!")
        else:
            print ("Failed to enable motors...")
    except rospy.ServiceException:
           print ("Enable service enable_motorscall failed: ")

def getPosition(odo):
    global position
    position = odo.pose.pose.position

    

def laser_callback(data):
    global position
    global direction
    global hover_check
    motor_command = Twist()
    speed = rospy.get_param("~speed", 1.5)
    turn = rospy.get_param("~turn", 1.5)
	
    if (data.ranges[len(data.ranges)/2])<2:
		    print'An obstacle detected'
		    direction='y'

    if not hover_check:
        speed = rospy.get_param("~speed", 0.1)
        turn = rospy.get_param("~turn", 0.1)
        hector_hover()

    x = moveBindings[direction][0]
    y = moveBindings[direction][1]
    z = moveBindings[direction][2]
    th =moveBindings[direction][3]
    global motor_command_publisher
    motor_command.linear.x = x*speed; 
    motor_command.linear.y = y*speed; 
    motor_command.linear.z = z*speed;
    motor_command.angular.x = 0; 
    motor_command.angular.y = 0;  
    motor_command.angular.z = th*turn
    motor_command_publisher.publish(motor_command)
    
    print 'Direction: ',direction
    print 'position(x,y,z) are : ',  position.x,position.y, position.z
    print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
    print '\n'
    time.sleep(0.2)


def drone_node():    
    rospy.init_node('drone_server')
    enableMotors()
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber('/ground_truth/state', Odometry, getPosition)
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size = 1000)

    
if __name__ == '__main__':
    drone_node()
    server = DroneServer()
    rospy.spin()





