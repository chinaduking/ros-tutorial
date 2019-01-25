#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from ninebot_gx.msg import status

import sys, select, termios, tty


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
	speed_loop_pub = rospy.Publisher('speed_loop_cmd', status, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0

	try:
		while(1):
			key = getKey()

			if key == 'w':
				speed = 0.2
				turn = 0
				twist = Twist()
				twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
				print('linear:',twist.linear.x,'       angular:',twist.angular.z);
				pub.publish(twist)
			elif key == 's':
				speed = -0.2
				turn = 0
				twist = Twist()
				twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
				print('linear:',twist.linear.x,'       angular:',twist.angular.z);
				pub.publish(twist)
			elif key == 'a':
				speed = 0
				turn = -0.2
				twist = Twist()
				twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
				print('linear:',twist.linear.x,'       angular:',twist.angular.z);
				pub.publish(twist)
			elif key == 'd':
				speed = 0
				turn = 0.2
				twist = Twist()
				twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0;
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
				print('linear:',twist.linear.x,'       angular:',twist.angular.z);
				pub.publish(twist)
			elif key == 'b':
				speed_loop = True
				speed_loop_pub.publish(speed_loop)
				print('speed loop is ture')
			elif key == 'u':
				speed_loop = False
				speed_loop_pub.publish(speed_loop)
				print('speed loop is false')
			elif key == '\x03':
				break
			else:
				print('key is not define')

	except KeyboardInterrupt:
		pass

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
