#!/usr/bin/env python

from pynput.keyboard import Key, Listener
import rospy
import sys
import time
from hex_movement import Hex
from std_msgs.msg import Int16MultiArray

class Control:
	"""
		Keyboard listener for press and release
	"""
	def on_press(self, key):
		if( isinstance(key, Key) ):
			if key == Key.esc:
				# Stop listener
				self.controlkey = 't'
				return False
				
			return True
			
		if(key.char in 'rtfg'):
			self.controlkey = key.char	
			
		elif(key.char in 'qweasdyxc13'):
			self.keypress = key.char
			

	def on_release(self, key):
	   self.keypress = '+'
	   return True


	def __init__(self):
		# Collect events until released
		self.listener = Listener(
				on_press=self.on_press,
				on_release=self.on_release)
		self.listener.start()
		
		self.keypress = '+'
		self.controlkey = 'r'
		

class Hex1Node:
	"""
		Hexapod subscribes for movement instruction.
		Data it receives through Int16MultiArray consists of power, angle and rotation 
	"""

	def callback(self, data):
		self.array = data.data

	def __init__(self):
		rospy.Subscriber("/algoritam/naredbe1", Int16MultiArray, self.callback) 
		self.array = [0, 0, 0]
	
class Hex2Node:
	"""
		Hexapod subscribes for movement instruction.
		Data it receives through Int16MultiArray consists of power, angle and rotation 
	"""

	def callback(self, data):
		self.array = data.data

	def __init__(self):
		rospy.Subscriber("/algoritam/naredbe2", Int16MultiArray, self.callback) 
		self.array = [0, 0, 0]


if __name__ == '__main__':
	control = Control()
	rospy.init_node('Hexapod')
	
	hex1Node = Hex1Node()
	hex2Node = Hex2Node()
	
	hex1 = Hex('192.250.9.9')
	hex2 = Hex('192.168.4.1')
	
	hex1running = False
	hex2running = False
	
	try:
		while 1:
			print ('hex 1 ' + str(hex1running) + ' hex2 ' + str(hex2running))
			
			if(control.listener.isAlive() ):
				print("alive")
			
			j = control.keypress
			
			"""hex1 movement"""
			if (control.controlkey == 'f'): 
				hex1running = True
				hex2running = False
				hex1.stop_hex()
			
			"""hex2 movement"""
			elif (control.controlkey == 'g'):
				hex1running = False
				hex2running = True
				hex2.stop_hex()

			"""automatic control of hex1 and hex2"""
			elif (control.controlkey == 'r'):
				print("Hex1 and Hex2 auto")
				hex1running = False
				hex2running = False
				hex1.automatic_control( hex1Node.array )
				hex2.automatic_control( hex2Node.array )
			
			"""stop hexapods"""	
			elif (control.controlkey == 't'):
				hex1.stop_hex()
				hex2.stop_hex()
				hex1.s.close()
				hex2.s.close()
				break
			  
			if hex1running:
				hex1.run(j)
				print("hex1")

			if hex2running:
				hex2.run(j)
				print("hex2")
	
			time.sleep(0.2)
		
	except rospy.ROSInterruptException:
		hex1.s.close()
		hex2.s.close()
		pass