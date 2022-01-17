#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist

bridge = CvBridge()

class StaircaseBot:
   
    def __init__(self):
        rospy.init_node('opencv_control', anonymous=True)
 
        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel',Twist, queue_size=10)
        self.img_subscriber = rospy.Subscriber('/jibo/camera1/image_raw',Image, self.image_callback)
        self.sonar_subscriber_1 = rospy.Subscriber('/sensor/sonar_front',Range, self.sonar_callback_1)
        self.sonar_subscriber_2 = rospy.Subscriber('/sensor/sonar_front_2',Range, self.sonar_callback_2)
 
        self.rate = rospy.Rate(10)
        self.error_dist = 0
        self.flag = 0
        self.sonar_data_1 = 0.0
        self.sonar_data_2 = 0.0

    def sonar_callback_1(self,sonar_msg):
        self.sonar_data_1 = sonar_msg.range
    def sonar_callback_2(self,sonar_msg):
        self.sonar_data_2 = sonar_msg.range

    def show_image(self,frame):
        cv.namedWindow("Image Window")
        lower = np.array([25, 52, 72])  
        upper = np.array([102, 255, 255])
        frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(frame_HSV, lower, upper)
        mask_Open = cv.morphologyEx(mask, cv.MORPH_OPEN, np.ones((10, 10)))
        mask_Close = cv.morphologyEx(mask_Open, cv.MORPH_CLOSE, np.ones((20, 20))) 
        mask_Perfect = mask_Close
        conts, h = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)[-2:] 
        for c in conts:
            areas = [cv.contourArea(c) for c in conts] 
            max_index = np.argmax(areas)
            cnt=conts[max_index]
            x, y, w, h = cv.boundingRect(cnt)
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv.circle(frame, (x + int(w*0.5), y + int(h*0.5)), 4, (0,0,255), -1) 
            (h1, w1) = frame.shape[:2] 
            cv.circle(frame, (w1//2, h1//2), 4, (255, 0, 0), -1)
            cv.circle(frame, (w1//2, h1//2), 15, (255, 0, 0), 3)
            error_d  = w1//2 - (x + int(w * 0.5))
            #print(error_d)
            y_low = 0
            if y + int(h*0.5)< h1//2:
                y_low = y + int(h*0.5)
            else:
                y_low = h1//2
            y_low -= 30
            cv.circle(frame, (w1//2,y_low), 3, (0,255,255), -1)
            cv.circle(frame, (x + int(w*0.5),y_low), 3, (0,255,255), -1)
            cv.line(frame, (x + int(w*0.5),y_low), (w1//2,y_low), (0,255,255), 2)
            # print("the error_d value is",error_d)
            # print("W1//2=",w1//2)
            self.error_dist = float(error_d)/float(w1//2)
            # print("the dist error over here is",self.error_dist)
        img = cv.cvtColor(frame, cv.COLOR_BGR2RGB)  
        cv.imshow("Image Window",img)
        cv.waitKey(1)


    def image_callback(self,img_msg):
        rospy.loginfo(img_msg.header)
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        self.show_image(cv_image)
 
    def move2goal(self):
 
        vel_msg = Twist()
        rospy.sleep(1)
        while not rospy.is_shutdown():
            print("the flag value is ",self.flag)
            print("The sonar_1 value is ",self.sonar_data_1)
            print("The sonar_2 value is ",self.sonar_data_2)
            if self.flag == 0:
                print("meoww111")
                vel_msg.linear.x = -0.5
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0
    
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                #print(self.error_dist)
                KP = 90
                #KP*self.error_dist
                vel_msg.angular.z = KP*self.error_dist
                # print("the dist error is ",self.error_dist)
                # print("the angular velocity is",KP*self.error_dist)
                self.velocity_publisher.publish(vel_msg)

            if self.sonar_data_1 <0.15 or self.sonar_data_2<0.15:
                print("meoww333")
                self.flag = 1
            if self.flag == 1:
                # print("meoww2222")
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0
    
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                diff1 = abs(self.sonar_data_1 - self.sonar_data_2)
                k_sonar = 20
                # print("The K-Sonar angular vel is", k_sonar*diff1)
                vel_msg.angular.z = k_sonar * diff1
                self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()
 
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
 
        rospy.spin()
 
# if _name_ == '_main_':
#     try:
#         x = StaircaseBot()
#         x.move2goal()
#     except rospy.ROSInterruptException:
#         pass

x = StaircaseBot()
x.move2goal()