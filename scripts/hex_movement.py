#!usr/bin/env python

import socket
import rospy
import sys
from struct import pack, unpack

class Hex():

    def __init__(self,ip):
        """Hexapod parameters"""
        
        self.IP = ip
        self.Port=80
        
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.IP, self.Port))

        self.power = 50
        self.angle = 0
        self.rotation = 0
        self.staticTilt = 0
        self.movingTilt = 0
        self.onoff = 1
        self.accX = 0
        self.accY = 0
        self.sliders1 = 50
        self.sliders2 = 25
        self.sliders3 = 0
        self.sliders4 = 0
        self.sliders5 = 0
        self.sliders6 = 50
        self.sliders7 = 0
        self.sliders8 = 0
        self.sliders9 = 0
        self.duration = 20
        self.array()

    def array(self):
        """Send parameters to hexapod"""
        self.paket=pack('cccbbbbbbbbbbbbbbbbbBB','P','K','T', self.power,(self.angle/2), self.rotation, self.staticTilt, self.movingTilt, self.onoff, self.accX, self.accY, self.sliders1, self.sliders2, self.sliders3, self.sliders4, self.sliders5, self.sliders6, self.sliders7, self.sliders8, self.sliders9, (self.duration/256), (self.duration%256))


    def manual_control(self, j):
        """Manaual hexapod control. Moves based on what is pressed on keyboard"""
        if (j == 'x'): #down
            self.power = 50
            self.angle = 180
            self.rotation = 0
           
        elif (j == 's'): #stop
            self.power = 0
            self.angle = 0
            self.rotation = 0
            
        elif (j == 'w'): #up
            self.power = 50
            self.angle = 0
            self.rotation = 0
        
        elif (j == 'd'): #right
            self.power = 50
            self.angle = 90
            self.rotation = 0
            
        elif (j == 'a'): #left
            self.power = 50
            self.angle = -90
            self.rotation = 0
            
        elif (j == '3'): #positive rotation
            self.power = 0
            self.angle = 0
            self.rotation = 50
          
        elif (j == '1'): #negative rotation
            self.power = 0
            self.angle = 0
            self.rotation = -50
            
        elif (j == 'e'): #up-right
            self.power = 50
            self.angle = 45
            self.rotation = 0
            
        elif (j == 'q'): #up-left
            self.power = 50
            self.angle = -45
            self.rotation = 0
           
        elif (j == 'y'): #down-left
            self.power = 50
            self.angle = -135
            self.rotation = 0
            
        elif (j == 'c'): #down-rigth
            self.power = 50
            self.angle = 135
            self.rotation = 0

        """Stop"""
        else: 
            self.power = 0
            self.angle = 0
            self.rotation = 0
        
        self.array()
    def automatic_control(self, array):
        """Hexapod automatic movement"""
        self.power = array[0]
        self.angle = array[1]
        self.rotation = array[2]
        self.array()
        self.s.send( self.paket )
     
    def stop_hex(self):
        """Stop hexapod movement"""
        self.power=0
        self.angle=0
        self.rotation=0
        self.array()
        self.s.send(self.paket)

    def run(self, j):
        """Runs hexapod manually"""
        self.manual_control(j)
        self.s.send(self.paket)
            