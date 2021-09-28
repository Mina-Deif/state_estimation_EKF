#!/usr/bin/env python
import rospy
import math
import numpy as np
from bicycle_model.msg import lateral_errors_msg
from bicycle_model.msg import coordinates_msg


#array will be used to define the path points
#import numpy as np
#path = np.array([[0,   0   ],
#		  [0.5 , 0.25],
#		  [0.75, 0.4 ],
#		  [1   , 0.55,],])
		  		  

# vehicle geometric parameters
L = 1.68        # Wheelbase in m
t = 1.1         # track width in m
lr = 0.68       # C.G to rear axle in m
rw = 0.3        # wheel radius in m
Kpp = 0.1       # pure pursuit proportional gain

#time parameter
time_step = 1/100  # in sec


#startt = -10.0
#endd = 0.0

msg_to_publish = lateral_errors_msg()


def callback(message):
#	global startt 
#	global endd

	vehicle_x = message.x					# current x coordinates relative to rear axle
	vehicle_y = message.y					# current y coordinates relative to rear axle	
	vehicle_theta = message.theta				# current vehicle heading
																									
	v = message.v 						# vehicle speeed
                   						
	msg_to_publish.ld = Kpp * v				# look ahead distance	
	
	flag = 0	
											
#	startt +=5.0
#	endd +=5.0
	
	for x in np.arange(0 ,1000, 0.01):
		y=10*math.sin(0.2*x)
		#y=1.0
		distance = math.sqrt((vehicle_x-x)**2+(vehicle_y-y)**2)	
		if ((distance > msg_to_publish.ld-0.05) and (distance < msg_to_publish.ld+0.05)):
				target_x = x
				target_y = y
				flag = 1
		


				
	delta_x = (target_x-vehicle_x)
	delta_y = (target_y-vehicle_y)
	
	if delta_x ==0 and delta_y > 0:				# at 90 degrees
		slope = math.tan( math.pi/2 )
	elif delta_x ==0 and delta_y < 0:
		slope = math.tan( math.pi*(3/2) )
	elif delta_x > 0 and delta_y == 0:
		slope = 0
	elif delta_x < 0 and delta_y == 0:
		slope = math.tan( math.pi )
	else:	
		slope = abs( delta_y / delta_x )
	
	angle = math.atan(slope)
	
	if delta_x < 0 and delta_y > 0:				# second quad
		angle = math.pi + angle	
	elif delta_x < 0 and delta_y < 0:				# third quad
		angle = math.pi - angle	
	elif delta_x > 0 and delta_y < 0:				# fourth quad
		angle = 2*math.pi - angle
	
	
	msg_to_publish.alpha =  angle - vehicle_theta 		# calculating heading error		
    
	rospy.loginfo(msg_to_publish)
	rospy.loginfo(flag)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)


rospy.init_node('path_generator')
sub = rospy.Subscriber('coordinates',coordinates_msg, callback)
pub = rospy.Publisher('lateral_control_errors', lateral_errors_msg, queue_size=10)
rate = rospy.Rate(1/time_step) # 1 Hz

rospy.spin()
