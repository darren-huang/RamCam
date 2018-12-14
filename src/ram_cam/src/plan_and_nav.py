#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf
import tf2_ros
import sys
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# RUNNING THIS FILE
# takes 5 arguements
# turtlebot_frame, board_grid_frame, board_block_frame, field_grid_frame, field_crate_frame
# should be:
# ar_marker_1 ar_marker_12 ar_marker_14 ar_marker_9 ar_marker_11



block_ar_tag_size = 5.6 # TODO: Change this value
board_intervals = 5.6/100 # TODO: Change this value
board_offset = 5.6/100 # TODO: Change this value
field_offset = 17.7/100 # TODO: Change this value
field_intervals = 17.7/100 # TODO: Change this value
field_offset = 30./100 
field_intervals = 30./100

# tb_center_to_edge = 7./100
# crate_center_to_edge = 11./100
push_offest = 24./100
safety_gap = push_offest*.5

backout_cmd_distance = push_offest*2
backout_speed = -0.3

K1 = 0.3
K2 = 1
K1,K2 = .75, 1.5
max_linear_speed = .15
linear_speed_default = .03 # always will at LEAST move linearly this fast
angular_turn_speed = .18 # always will at LEAST turn this fast
sigma = 0.01
sigma = 0.0025 # tolerance for distance from waypoint
# sigma_proportional_ctrl = 0.07
sigma_ang = 0.03
#sigma_ang_rad = 0.05

class backout_cmd_obj_class:
  def finished(self, tb_coord, crate_coord):
    dist_to_crate = (((tb_coord[0] - crate_coord[0]) ** 2) + ((tb_coord[1] - crate_coord[1]) ** 2)) ** 0.5
    return dist_to_crate >= backout_cmd_distance
backout_cmd = backout_cmd_obj_class()

def translate(coord, board_bool):
  if board_bool:
    return [round((coord[x] - board_offset) / board_intervals) for x in range(2)]
  else:
    return [round((coord[x] - field_offset) / field_intervals) for x in range(2)]

def untranslate(coord, board_bool):
  if board_bool:
    return [(coord[x]*board_intervals) + board_offset for x in range(2)]
  else:
    return [(coord[x]*field_intervals) + field_offset for x in range(2)]

def move_frame(coord, orig_f, new_f, listener):
  # listener = tf.TransformListener()
  time_0 = rospy.Time(0)
  listener.waitForTransform(orig_f, new_f, time_0,rospy.Duration(4.0))
  laser_point=PointStamped()
  laser_point.header.frame_id = orig_f
  laser_point.header.stamp =time_0
  laser_point.point.x=coord[0]
  laser_point.point.y=coord[1]
  laser_point.point.z=0.0
  p=listener.transformPoint(new_f,laser_point)
  return [p.point.x, p.point.y]


def zero_vec():
  pub_string = Twist()
  angular_v = Vector3()
  linear_v = Vector3()
  linear_v.x, linear_v.y, linear_v.z = 0, 0, 0 
  angular_v.x, angular_v.y, angular_v.z = 0, 0, 0
  pub_string.linear = linear_v
  pub_string.angular = angular_v
  return pub_string

def cap_speed(speed):
  return max(min(speed, max_linear_speed), -max_linear_speed)

def calc_waypoints(tb_coord, crate_coord, destination_coord):
  waypoints = []
  #TODO deal with divide by zeros? unlikeley tho

  # deal with 0th coord first
  push_d0 = destination_coord[0] - crate_coord[0]
  direction0 = round(push_d0 / abs(push_d0))
  pre_push0_coord = [(-direction0 * push_offest) + crate_coord[0] + safety_gap, crate_coord[1]] #TODO ensure safety gap is accounted for
  print("PRE_PUSH_COORD:", pre_push0_coord)

  # note that pre_push0_coord is our 1st destination our tb must reach without touching the crate
  if crate_coord[0] - pre_push0_coord[0] != 0:
    pp0c_to_crate_direction = round((crate_coord[0] - pre_push0_coord[0])/abs(crate_coord[0] - pre_push0_coord[0]))
  else:
    pp0c_to_crate_direction = 1
  delta_0 = tb_coord[0] - pre_push0_coord[0]

  if pp0c_to_crate_direction * (delta_0) > 0: #crate and turtlebot on same side w.r.t. prepush point
    #some positioning waypoints to prevent hitting the crate
    first_waypoint_possibilities = [[tb_coord[0], pre_push0_coord[1] + push_offest + safety_gap],
                                    [tb_coord[0], pre_push0_coord[1] - (push_offest + safety_gap)]] #position the y-axis s.t. we don't hit the crate when moving along x-axis
    first_waypoint = min(first_waypoint_possibilities, key=lambda coord: abs(coord[1] - tb_coord[1]))
    waypoints.append(first_waypoint)
    second_waypoint = [pre_push0_coord[0], first_waypoint[1]]
    waypoints.append(second_waypoint)
  #otherwise go directly to the prepush point
  waypoints.append(pre_push0_coord)

  push_waypoint = [0, 0]
  if (pp0c_to_crate_direction > 0):
    push_waypoint = [destination_coord[0] - push_offest, pre_push0_coord[1]]
  else: 
    push_waypoint = [destination_coord[0] + push_offest, pre_push0_coord[1]]

  waypoints.append(push_waypoint)

  post_push_direction = round((destination_coord[1] - crate_coord[1]) / abs(destination_coord[1] - crate_coord[1]))

  #right after first push
  waypoints.append(backout_cmd) #TODO check this is added after the backout cmd stuffs

  back_out_waypoint = [push_waypoint[0] - (pp0c_to_crate_direction * (push_offest)), push_waypoint[1] - (post_push_direction * (safety_gap + push_offest))]
  waypoints.append(back_out_waypoint)

  pre_post_waypoint = [destination_coord[0], back_out_waypoint[1]]

  waypoints.append(pre_post_waypoint)

  final_push_waypoint = [destination_coord[0], destination_coord[1] - (post_push_direction * (push_offest))]

  waypoints.append(final_push_waypoint)

  # last_back_out_waypoint = [destination_coord[0], final_push_waypoint[1] - (post_push_direction * (safety_gap + push_offest))]

  # waypoints.append(last_back_out_waypoint)

  waypoints.append(backout_cmd) #TODO check this is added after the backout cmd stuffs

  return waypoints

#Define the method which contains the main functionality of the node.
def controller(turtlebot_frame, board_grid_frame, board_block_frame, field_grid_frame, field_crate_frame):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame

  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - target_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  tranform_listener = tf.TransformListener()
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz
  waypoints = [] # xy coord is relative field_grid_frame ([1st 2nd 3rd etc.])
  prev_block_coord = None

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      print(" ")
      time_0 = rospy.Time(0)
      trans = tfBuffer.lookup_transform(board_grid_frame, board_block_frame, time_0)
      stamp = trans.transform.translation
      block_coord = [stamp.x, stamp.y]
      trans = tfBuffer.lookup_transform(field_grid_frame, turtlebot_frame, time_0)
      stamp = trans.transform.translation
      tb_coord = [stamp.x, stamp.y]
      trans = tfBuffer.lookup_transform(field_grid_frame, field_crate_frame, time_0)
      stamp = trans.transform.translation
      crate_coord = [stamp.x, stamp.y]

      #Check for block movement
      lattice_block_coord = translate(block_coord, True)
      print("block coord", block_coord)
      print("block lattice coord", lattice_block_coord)
      lattice_crate_coord = translate(crate_coord, False)
      print("crate coord", crate_coord)
      print("crate lattice coord", lattice_crate_coord)
      #TODO DELETE: this is just for testing
      lattice_tb_coord = translate(tb_coord, False)
      print("tb coord", tb_coord)
      print("tb lattice coord", lattice_tb_coord)
      print("distance_y:", crate_coord[1] - tb_coord[1])
      if prev_block_coord is None:
        #prev_block_coord = lattice_block_coord
        raw_input("type 'enter' when ready")
      block_moved = (lattice_block_coord != prev_block_coord) 

      #if block moves, plan new path ie. update waypoints
      if block_moved:
        # psuedo_waypoint = (0, 0)#plan(lattice_block_coord, tb_coord, crate_coord)
        prev_block_coord = lattice_block_coord
        destination_coord = untranslate(lattice_block_coord, False)
        waypoints = calc_waypoints(tb_coord, crate_coord, destination_coord) #Returns True waypoints, NOT lattice poitns
        print("In block moved function: ", block_moved)

      # check if waypoint reached
      err = None
      if waypoints:
        wp = waypoints[0]
        # wp = untranslate(wp, False)
        if isinstance(wp, backout_cmd_obj_class): # this is a backout command
          if wp.finished(tb_coord, crate_coord):
            waypoints = waypoints[1:]
        else: # this is a coordinate
          err = ((tb_coord[0] - wp[0]) ** 2) + ((tb_coord[1] - wp[1]) ** 2)
          print("Err is: ", err)
          if err <= sigma:
            waypoints = waypoints[1:]

      # waypoints = [untranslate(lattice_block_coord, False)] # TODO
      if waypoints: # move to next waypoint
        wp = waypoints[0]
        if isinstance(wp, backout_cmd_obj_class): # just backout
          control_command = [backout_speed, 0]
        else: #wp is actual coordinates
          # wp = crate_coord # TODO: Uncomment
          # wp_untraslated = untranslate(wp, False)
          wp_in_tb_frame = move_frame(wp, field_grid_frame, turtlebot_frame, tranform_listener)
          # angle_to_wp = np.arctan(wp_in_tb_frame[1]/wp_in_tb_frame[0])

          print("waypoint is now: ", wp_in_tb_frame)
          # print("angle to waypoint: ", angle_to_wp)
          if abs(wp_in_tb_frame[1]) <= sigma_ang: # roughly lined up 
          # if abs(angle_to_wp) <= sigma_ang_rad:
            direction = wp_in_tb_frame[0] / abs(wp_in_tb_frame[0]) if wp_in_tb_frame[0] != 0 else 0
            control_command = [(linear_speed_default * direction) + (K1 * wp_in_tb_frame[0]), 0] #Proportional linear control
          else: # only turn
            direction = wp_in_tb_frame[1] / abs(wp_in_tb_frame[1]) if wp_in_tb_frame[1] != 0 else 0
            control_command = [0, (angular_turn_speed * direction) + (K2 * wp_in_tb_frame[1])] #Proportional angle control

        #control_command = [K1 * wp_in_tb_frame[0], K2 * wp_in_tb_frame[1]] # OLD CONTROL
        # control_command will be [linear_speed, angular_speed]
        control_command[0] = cap_speed(control_command[0]) #TODO Check the cap speed
        pub_string = Twist()
        angular_v = Vector3()
        linear_v = Vector3()
        linear_v.x, linear_v.y, linear_v.z = control_command[0], 0, 0 
        angular_v.x, angular_v.y, angular_v.z = 0, 0, control_command[1]
        pub_string.linear = linear_v
        pub_string.angular = angular_v
        print("control command:", control_command)
        control_command = pub_string
      else:
        control_command = zero_vec()

      ##### end calc #####
      print(waypoints)
      # print("control command", control_command)
      # control_command = zero_vec() # TODO REMOVe



      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
      print("GOT ERROR:", e)
      control_command = zero_vec()
      pub.publish(control_command)
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  try:
    controller(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
  except rospy.ROSInterruptException:
    pass
