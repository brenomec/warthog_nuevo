#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
import rospy
import math
import roslib
import sys
import cv2 as cv
import cv2.aruco as aruco
import numpy as np

class Warthog():

    def __init__(self, odom_topic, cmd_vel_topic, tolerance=0.3):
        # Creates a node with name 'warthog_controller' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('warthog_controller', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscriber to the Odom data:
        self._odomSubscriber = rospy.Subscriber("/odometry/filtered", Odometry, self.get_position_odom)

        # Internal Variables iniciation
        self._odomTopic = odom_topic
        self._cmd_velTopic = cmd_vel_topic
        self._tolerance = tolerance
        self._pose = [0, 0, 0]
        self.goal_pose = [0, 0]
        self._end = False

        # Gets the inputs from the user
        self.goal_pose[0] = input("Set your x goal: ")
        self.goal_pose[1] = input("Set your y goal: ")

    # Function that converts the Odometry data to Pose type msg
    def get_position_odom(self, msg):
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self._pose[2] = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # Function to move the robot
    def move2goal(self):
        velocityCommand = Twist()
        # Calculation of the distances and angle between the two points
        distanceX = abs(self.goal_pose[0] - self._pose[0])
        distanceY = abs(self.goal_pose[1] - self._pose[1])
        teta = math.atan2((self.goal_pose[1] - self._pose[1]), (self.goal_pose[0] - self._pose[0]))
        if((distanceX and distanceY) >= self._tolerance):
            velocityCommand.linear.x = 0.5
            erroAngle = (teta - self._pose[2])
            if(erroAngle > 3.1415):
                erroAngle -= 2*3.1415
            elif(erroAngle < -3.1415):
                erroAngle += 2*3.1415
            velocityCommand.angular.z = erroAngle * 0.5
        else :
            velocityCommand.angular.z = 0
            velocityCommand.linear.x = 0 
            self.velocity_publisher.publish(velocityCommand)      
        self.velocity_publisher.publish(velocityCommand)

class image_detection():

    def __init__(self):
        
        # Creates a node with name 'warthog_controller' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('warthog_controller', anonymous=True)
        
        # Create the ROS publisher and Subscriber
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/warthog/camera1/image_raw",Image,self.callback)
        self.id = []
        # Initial Variables
        self.bridge = CvBridge()

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        # Load the image
        image = cv_image
    
        # Converting the image to a grayscale
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        # Loading the aruco original dictionary for comparison
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        # Specific Parameters generation
        parameters =  aruco.DetectorParameters_create()

        # Lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Function that displays the id and the corners to the image
        gray = aruco.drawDetectedMarkers(image, corners, ids)

        # show the images
        cv.imshow('id_detection',gray)
        cv.waitKey(3)

        #
        self.id = ids

        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
          print(e)

    def getIds(self):
        if (not self.id):
            return list()
        return self.id

class every():

    def __init__(self):
        self.id = image_detection()
        self.warthog = Warthog("/odometry/filtered", "/cmd_vel", 0.3)
        self.encontrado = False

    def move(self):
        lista = self.id.getIds()
        type_list = type(lista)
        #print (type_list) 
        if not self.encontrado:
            if 4 in lista:
                print ("Tag recognized")
                self.encontrado = True
            else:
                print ("No tag recognized")
        else:
            self.warthog.move2goal()

if __name__ == "__main__":
    E = every()
    while(not rospy.is_shutdown()):
        E.move()
        whileRate = rospy.Rate(50)
        whileRate.sleep()