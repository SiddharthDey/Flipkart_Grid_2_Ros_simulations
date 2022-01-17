import rospy
import numpy as np
from sensor_msgs.msg import Range
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class StaircaseBot:
   
    def __init__(self):
        rospy.init_node('climb_stairs', anonymous=True)
 
        # self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        # self.img_subscriber = rospy.Subscriber('/mybot/camera1/image_raw',Image, self.image_callback)
        self.bot_move_publisher = rospy.Publisher('/World_BaseLink_joint_position_controller/command',Float64, queue_size=10)
        self.leg1_publisher = rospy.Publisher('/BaseLink_Leg1_joint_position_controller/command',Float64, queue_size=10)
        self.leg2_publisher = rospy.Publisher('/BaseLink_Leg2_joint_position_controller/command',Float64, queue_size=10)
        self.leg3_publisher = rospy.Publisher('/BaseLink_Leg3_joint_position_controller/command',Float64, queue_size=10)
        self.leg4_publisher = rospy.Publisher('/BaseLink_Leg4_joint_position_controller/command',Float64, queue_size=10)
        self.leg5_publisher = rospy.Publisher('/BaseLink_Leg5_joint_position_controller/command',Float64, queue_size=10)
        self.leg6_publisher = rospy.Publisher('/BaseLink_Leg6_joint_position_controller/command',Float64, queue_size=10)
        self.sonar_subscriber = rospy.Subscriber('/sensor/sonar_front',Range, self.sonar_callback)


        self.rate = rospy.Rate(10)
        self.current_bot_pose = 0.0
        self.sonar_data = 0.0
        self.flag1 = 0
        self.flag2 = 0

    def sonar_callback(self,sonar_msg):
        self.sonar_data = sonar_msg.range

    def move2goal(self):
 
        # leg1_msg = Float64()
        # leg2_msg = Float64()
        # leg3_msg = Float64()
        # leg4_msg = Float64()
        # leg5_msg = Float64()
        # leg6_msg = Float64()
        # bot_move_msg = Float64()

        
        while not rospy.is_shutdown():
            # leg1_msg = 0.0
            # self.leg1_publisher.publish(Float64(leg1_msg))
            if self.flag1==0 and self.sonar_data > 0.15:
                self.bot_move_publisher.publish(Float64(self.current_bot_pose))
                # print("meow2222")
                self.current_bot_pose -= 0.05
                
                # if self.sonar_data < 0.15 and self.sonar_data > 0.0:
                #     self.flag1 = 1
                    
            k1 = 0
            if self.sonar_data < 0.15 and self.sonar_data > 0.0 and self.flag1==0:
                leg1_msg = 0.165
                leg2_msg = 0.165
                self.leg1_publisher.publish(Float64(leg1_msg))
                self.leg2_publisher.publish(Float64(leg2_msg))
                rospy.sleep(0.5)
                self.flag1 = 1
                # k1 += 1
                
                # if k1 > 250:
                #     self.flag1 = 1

            if self.flag1==1 and self.sonar_data > 0.38:
                self.bot_move_publisher.publish(Float64(self.current_bot_pose))
                self.current_bot_pose -= 0.05
                # if self.sonar_data <= 0.38 and self.sonar_data > 0.0:
                #     self.flag1 = 3
                #     print("meoww4444")
                    

            k2 = 0
            if self.flag1==1 and self.sonar_data <= 0.38:
                leg3_msg = 0.165
                leg4_msg = 0.165
                self.leg3_publisher.publish(Float64(leg3_msg))
                self.leg4_publisher.publish(Float64(leg4_msg))
                rospy.sleep(0.5)
                self.flag1 = 2
                # k2+=1
                # if k2==250:
                #     self.flag1 = 2
            
            if self.flag1==2 and self.sonar_data > 0.2:
                self.bot_move_publisher.publish(Float64(self.current_bot_pose))
                self.current_bot_pose -= 0.01
                print(self.sonar_data)
            
            if self.flag1==2 and self.sonar_data <= 0.2:
                leg5_msg = 0.165
                leg6_msg = 0.165
                self.leg5_publisher.publish(Float64(leg5_msg))
                self.leg6_publisher.publish(Float64(leg6_msg))
                rospy.sleep(0.5)
                self.flag1 = 3

            if self.flag1==3 and self.sonar_data > 0.1:
                self.bot_move_publisher.publish(Float64(self.current_bot_pose))
                self.current_bot_pose -= 0.01
                print(self.sonar_data)
            
            
            self.rate.sleep()
        
 
        
 
        rospy.spin()

# if _name_ == '_main_':
#     try:
#         x = StaircaseBot()
#         x.move2goal()
#     except rospy.ROSInterruptException:
#         pass

x = StaircaseBot()
x.move2goal()