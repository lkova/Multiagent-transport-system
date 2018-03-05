import rospy
from geometry_msgs.msg import Pose2D, Pose
from math import *
import time
import socket
import sys
from struct import *
from std_msgs.msg import Int16MultiArray


class HexaGuide():
    """Class which contains implementation of hexapod movement alghorithm"""

    def __init__(self):
        self.pose1      = Pose2D()
        self.pose2      = Pose2D()
        self.marker     = Pose2D()
        self.marker2    = Pose2D()

        """Subscribers on marker position"""
        rospy.Subscriber("/pose_node/pose_id1", Pose2D, self.pose_callback1)
        rospy.Subscriber("/pose_node/pose_id2", Pose2D, self.pose_callback2)
        rospy.Subscriber("/pose_node/pose_id15", Pose2D, self.pose_callback15)
        rospy.Subscriber("/pose_node/pose_id4", Pose2D, self.pose_callback4)

        """Publisher for hexapod movement"""
        self.pub1 = rospy.Publisher("/algoritam/naredbe1", Int16MultiArray, queue_size = 10)
        self.pub2 = rospy.Publisher("/algoritam/naredbe2", Int16MultiArray, queue_size = 10)

        
    def pose_callback1(self, data):                        
        self.marker = data;

    def pose_callback2(self, data):                        
        self.pose1 = data

    def pose_callback15(self, data):
        self.pose2 = data

    def pose_callback4(self, data):
        self.marker2 = data
        
    def rotate_hexapods(self, sign1, sign2, sign_rot):
        """Rotation of hexapods in direction of tunnel"""

        if self.pose1.x < self.pose2.x:
            sign_1 = -1
            sign_2 = 1
            sign_rot = -1
            if self.pose2.theta - self.pose1.theta > 5:
                rotation2 = sign_rot * 27
                rotation1 = sign_rot * 22
                power2 = sign2 * 37
                power1 = sign1 * 37
                angle1 = -90
                angle2 = 90

            elif self.pose1.theta - self.pose2.theta > 5:
                rotation2 = sign_rot * 17
                rotation1 = sign_rot * 23
                power2 = sign2 * 27
                power1 = sign1 * 32
                angle1 = -90
                angle2 = 90
            
        else:
            sign_1 = 1
            sign_2 = -1
            sign_rot = 1

            if self.pose2.theta - self.pose1.theta > 5:
                rotation2 = sign_rot * 27
                rotation1 = sign_rot * 22
                power2 = sign2 * 37
                power1 = sign1 * 37
                angle1 = 90
                angle2 = -90


            elif self.pose1.theta - self.pose2.theta > 5:
                rotation2 = sign_rot * 23
                rotation1 = sign_rot * 17
                power2 = sign2 * 30
                power1 = sign1 * 27
                angle1 = 90
                angle2 = -90

        while not(rospy.is_shutdown()) and abs(self.pose1.theta) > 3:
            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)

    def move_from_tunnel(self):
        """Moving hexapods away from tunnel so it would avoid colision"""

        if ((self.pose2.x < (self.marker.x + 0.25) and self.pose2.x > self.marker.x) or (self.pose2.x > (self.marker2.x - 0.25) and self.pose2.x < self.marker2.x) and abs(self.pose1.theta) > 90
            or (self.pose2.y < (self.marker2.y + 0.25) and self.pose2.y < self.pose1.y and abs(self.pose1.theta) < 90) or (self.pose2.y < (self.marker.y - 0.25) and self.pose2.y > self.pose1.y and abs(self.pose1.theta) > 90)):                                                                                 
            angle1 = 180
            angle2 = 180
            power1 = 50
            power2 = 50
            rotation1 = 0
            rotation2 = 0
            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)

        if ((self.pose1.x < (self.marker.x + 0.25) and self.pose1.x > self.marker.x) or (self.pose1.x > (self.marker.x - 0.25) and self.pose1.x < self.marker2.x) and abs(self.pose1.theta) > 90
            or (self.pose1.y < (self.marker2.y + 0.25) and self.pose1.y < self.pose2.y and abs(self.pose1.theta) > 90) or (self.pose1.y < (self.marker.y - 0.25) and self.pose1.y > self.pose2.y and abs(self.pose1.theta) < 90)):                                                                                 
            angle1 = 0
            angle2 = 0
            power1 = 50
            power2 = 50
            rotation1 = 0
            rotation2 = 0
            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)

        time.sleep(3)


    def publish_instructions(self, power1=0, power2=0, angle1=0, angle2=0, rotation1=0, rotation2=0):
        """publishing movement parameters for hexapod"""
        hex1_array = Int16MultiArray(data=[self.power1, self.angle1, self.rotation1])
        hex2_array = Int16MultiArray(data=[self.power2, self.angle2, self.rotation2])
        self.pub1.publish(hex1_array)
        self.pub2.publish(hex1_array)
        time.sleep(0.2)


    def align_hexapods(self, power):
        """Aligns hexapod2 with hexapod1 for synchronous movement"""
        
        power1 = power

        while (abs(self.pose1.theta) > 7) or ( abs(self.pose2.theta - self.pose1.theta) > 7):
            print "ispravljam kut"
            power1 = 0
            power2 = 0
            rotation1 = -int(self.pose1.theta * (100. / 180))
            rotation2 = -int(self.pose2.theta * (100. / 180))
            angle1 = 0
            angle2 = 0

            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)

        while ( (self.pose1.y - self.pose2.y) > 0.49 or  (self.pose1.y - self.pose2.y) < 0.41 ):
            print "poravnava po y"
            if ( (self.pose1.y - self.pose2.y) > 0.49 ):
                angle2 = 0
            else: 
                angle2 = 180
            power1 = 0
            power2 = 10
            rotation1 = 0
            rotation2 = 0
            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)

        while (abs(self.pose1.x - self.pose2.x) > 0.02):
            print "ravna po x"
            rotation1 = 0
            rotation2 = 0
            angle1 = 0
            angle2 = copysign( 90 , self.pose2.x - self.pose1.x)
            power1 = 0
            power2 = 20
                
            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)

                
    def run(self):
        """Runs hexapods until they reach their goal, until they pass tunnnel"""

        self.move_from_tunnel()
        self.rotate_hexapods()
    
        """Moves hexapods by y coordinate to align with the tunnel"""
        while not(rospy.is_shutdown()):
            self.align_hexapods(0)
       
            deltay1 = self.marker.y - self.pose1.y
            deltay2 = self.pose2.y - self.marker2.y
            if (deltay1 > 0.2 or deltay2 > 0.2):
                power1 = 0
                power2 =  0
                break 
            else:
                if (self.pose1.y > (((self.marker.y + self.marker2.y)/2) + 0.225)):
                    angle1 = 0
                    angle2 = 0
                else:
                    angle1 = 180
                    angle2 = 180
                power1 = 35
                power2 = 35
            rotation1 = 0
            rotation2 = 0

            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)
        
        """Align hexapods after task"""
        self.align_hexapods(10)
          
        
        """Moves hexapods by x coordinate to align with the tunnel"""
        while not(rospy.is_shutdown()):
            self.align_hexapods(0)
    
            deltax = abs(((self.marker2.x + self.marker.x)/2) - self.pose1.x)
            if (((self.marker2.x + self.marker.x)/2) > self.pose1.x):
                angle1 = -90
                angle2 = -90
            else:
                angle1 = 90
                angle2 = 90
            if (deltax < 0.05):
                power1  = 0
                power2  =  0
                break
            else: 
                power1 = 35
                power2 = 35

            rotation1 = 0
            rotation2 = 0

            self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)
        
        """Align hexapods after task"""
        self.align_hexapods(10)
                
        """Hexapods goes through tunnel"""
        while not(rospy.is_shutdown()):               
            if (self.pose1.y > self.marker2.y):                                 
                while (self.pose1.y > (self.marker.y - 0.2)):
                    self.align_hexapods(0)
                    angle1 = 180
                    angle2 = 180
                    power1 = 35
                    power2 = 35
                    rotation1 = 0
                    rotation2 = 0
                    self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)
                    
            else:                                                                               
                while (self.pose2.y < (self.marker2.y + 0.2)):
                    while (abs(self.pose1.theta) > 7) or ( abs(self.pose2.theta - self.pose1.theta) > 7 ):
                        self.align_hexapods(0)
                    angle1 = 180
                    angle2 = 180
                    power1 = 35
                    power2 = 35
                    rotation1 = 0
                    rotation2 = 0
                    self.publish_instructions(power1, power2, angle1, angle2, rotation1, rotation2)
            break
        
        self.publish_instructions()
        
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('Control', anonymous = True)
        Hex = HexaGuide()
        Hex.publish_instructions()
        Hex.run() 
        rospy.spin()
    except (rospy.ROSInterruptException,KeyboardInterrupt):
        Hex.publish_instructions()
        s1.close()
        s2.close()
        sys.exit()
