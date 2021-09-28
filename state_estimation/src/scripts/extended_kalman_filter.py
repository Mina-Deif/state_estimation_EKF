#!/usr/bin/env python
import rospy
import math
import numpy as np
from extended_kalman_filter.msg import coordinates_msg
from extended_kalman_filter.msg import velocity_msg
from extended_kalman_filter.msg import estimated_coordinates_msg



# vehicle geometric parameters
L = 1.68        # Wheelbase in m

#time parameter
time_step = 1/100  # in sec


msg_to_publish = estimated_coordinates_msg()



v=0
delta = 0
theta = 0
sensor_x = 0
sensor_y = 0
sensor_theta = 0


x = np.array(  [ [v*math.cos(theta)*time_step  ],
                 [v*math.sin(theta)*time_step  ],
                 [v*math.tan(delta)*time_step/L], ])

h = np.array(  [ [sensor_x    ],
                 [sensor_y    ],
                 [sensor_theta], ])


G = np.array(  [[1, 0, -v*math.sin(theta)*time_step],
                [0, 0,  v*math.cos(theta)*time_step],
                [0, 2,             1               ], ])

H = np.array(  [[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1], ])

L = np.array(  [[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1], ])

M = np.array(  [[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1], ])

Q = np.array(  [[0.1, 0, 0],   # variance in prediction noise
                [0,   1, 0],
                [0,   0, 1], ])

previous_estimated_x = np.array(  [ [0],
                                    [0],
                                    [0], ])
 
previous_estimated_P = np.array(  [ [0.01, 0, 0],
                                    [0,    1, 0],
                                    [0,    0, 1], ])


#===============================================================
def callback1(message):
	
	global sensor_x
	global sensor_y
	global sensor_theta
	
	sensor_x = message.x
	sensor_y = message.y
	sensor_theta = message.theta
	
	h = [ [sensor_x    ],
	      [sensor_y    ],
	      [sensor_theta], ]

def callback2(message):
	global G
	global H
	global L
	global M
	global Q
	global previous_estimated_x
	global previous_estimated_P
	global x
	global h
	global v
	global delta
	global theta
	
	v=message.velocity
	delta = message.delta
	theta = v*math.tan(delta)*time_step/L
	
	x = [ [v*math.cos(theta)*time_step ],
              [v*math.sin(theta)*time_step ],
              [		theta       ], ]

	G =    [[1, 0, -v*math.sin(theta)*time_step],
	        [0, 0,  v*math.cos(theta)*time_step],
	        [0, 2,             1               ], ]

######################################### 
# prediction step

	predicted_x = previous_predicted_x + x
 
	predicted_P = G @ previous_estimated_P @ G.T + L @ Q @ L.T;

######################################### 
# correction step determining gain factor
 
	K = ( predicted_P @ H.T ) * linalg.pinv(H @ predicted_P @ H.T + M @ R @ M.T);

########################################### 
# corrected output for position and velocity

	estimated_x = predicted_x + K @ (yk - h);
	estimated_P = (1-k @ H) @ predicted_P

########################################### 
# update step

	previous_predicted_x = predicted_x
	previous_estimated_x = estimated_x
	previous_estimated_P = estimated_P
	
###########################################

	msg_to_publish.estimated_x = estimated_x[0]
	msg_to_publish.estimated_y = estimated_x[1]
	msg_to_publish.estimated_theta = estimated_x[2]
	msg_to_publish.v=v
	
	rospy.loginfo(msg_to_publish)
	rospy.loginfo("\n")
    
    
	pub.publish(msg_to_publish)
	
	
rospy.init_node('extende_kalman_filter')
sub1 = rospy.Subscriber('coordinates', coordinates_msg, callback1)
sub2 = rospy.Subscriber('steering_input', velocity_msg, callback2)
pub = rospy.Publisher('estimation', estimated_coordinates_msg, queue_size=10)
rate = rospy.Rate(1/time_step) # 1 Hz

rospy.spin()
