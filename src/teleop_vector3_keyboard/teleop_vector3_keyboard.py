#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Vector3

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Vector3!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1),
	       }

moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,1),
		'm':(-1,-1),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 1

def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('robot1/base', Vector3, queue_size=10)
	rospy.init_node('teleop_vector3_keyboard')

	x = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			vector3 = Vector3()
			if key=='8':
				vector3.x = 0.0; vector3.y = -0.1; vector3.z = 0.1
			elif key=='2':
				vector3.x = 0; vector3.y = 0.1; vector3.z = -0.1
			elif key=='6':
				vector3.x = 0.05; vector3.y = -0.025; vector3.z = -0.025
			elif key=='4':
				vector3.x = -0.05; vector3.y =0.025; vector3.z =0.025
			elif key=='7':
				vector3.x = -0.025; vector3.y =-0.025; vector3.z =-0.025
			elif key=='9':
				vector3.x = 0.025; vector3.y =0.025; vector3.z =0.025
			elif (key == '\x03'):
				break
			else:
				vector3.x = 0; vector3.y = 0; vector3.z = 0
			pub.publish(vector3)

	except:
		print e

	finally:
		vector3 = Vector3()
		vector3.x = 0; vector3.y = 0; vector3.z = 0
		pub.publish(vector3)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


