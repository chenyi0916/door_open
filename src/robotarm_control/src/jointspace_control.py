#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from tf import transformations

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def interp5rd(qs, qvs, qas, qf, qvf, qaf,tf,dt):
  
  n = len(qs)
  k = np.floor(tf/dt).astype(int) + 1

  a0 = np.zeros(n)
  a1 = np.zeros(n)
  a2 = np.zeros(n)
  a3 = np.zeros(n)
  a4 = np.zeros(n)
  a5 = np.zeros(n)
  for i in range(n):
    a0[i] = qs[i]
    a1[i] = qvs[i]
    a2[i] = qas[i]/2.0
    a3[i] = (20*(qf[i] - qs[i]) - (8*qvf[i] + 12*qvs[i])*tf + (qaf[i] - 3*qas[i])*tf*tf)/(2*pow(tf,3))
    a4[i] = (-30*(qf[i] - qs[i]) + (14*qvf[i] + 16*qvs[i])*tf - (2*qaf[i] - 3*qas[i])*tf*tf)/(2*pow(tf,4))
    a5[i] = (12*(qf[i] - qs[i]) - (60*qvf[i] + 6*qvs[i])*tf + (qaf[i] - qas[i])*tf*tf)/(2*pow(tf,5))

  qq = np.zeros([k,n])
  qv = np.zeros([k,n])
  qa = np.zeros([k,n])
  t_seq = np.linspace(0,tf,k)
  for i in range(k):
    t = t_seq[i]
    for j in range(n):
      qq[i,j] = a0[j] + a1[j]*t + a2[j]*pow(t,2) + a3[j]*pow(t,3) + a4[j]*pow(t,4) + a5[j]*pow(t,5)
      qv[i,j] = a1[j] + 2*a2[j]*t + 3*a3[j]*pow(t,2) + 4*a4[j]*pow(t,3) + 5*a5[j]*pow(t,4)
      qa[i,j] = 2*a2[j] + 6*a3[j]*t + 12*a4[j]*pow(t,2) + 20*a5[j]*pow(t,3)
  return [qq,qv,qa]


class jointspace_control:
    def __init__(self):
       
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('jointspace_control', anonymous=True)
 
        arm = moveit_commander.MoveGroupCommander('arm1')
        gripper = moveit_commander.MoveGroupCommander('gripper1')

        display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
)
        

        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.01)
        
        # arm.set_named_target('rest')
        rest_position = [-0.000047, -1.570638,1.499957, -8.1e-5, 0.399982,-7.8e-5]
        arm.set_joint_value_target(rest_position)
        arm.go()
        # rospy.sleep(2)
        
        gripper.set_named_target('open')
        gripper.go()
        rospy.sleep(1)
         
        print("Move to pre_grasp pose")
        joint_positions = [-0.364979, 1.089672, -0.938418, 1.944205, 0.394166, -0.403364]
        joint_velocity = [0, 0, 0, 0, 0, 0]
        joint_acc = [0, 0, 0, 0, 0, 0]
        tf = 10
        dt = 0.1

        [qq, qv, qa] = interp5rd(rest_position,joint_velocity,joint_acc, joint_positions,joint_velocity,joint_acc,tf,dt)

        for i in range(100):
          target_position = qq[i]
          arm.set_joint_value_target(target_position)
          arm.go()
        

        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        rospy.sleep(1)
        
        print("gripper close")
        gripper.set_joint_value_target([0.95, 0.95])
        gripper.go()
        rospy.sleep(1)

        # radius = 0.64903119395423438
        # lenth = 14
        # th = 0.05
        # wpose = arm.get_current_pose().pose
        # grasppose = wpose
        # waypoints = []
      
        # for i in range(lenth):
        #   quaternion = transformations.quaternion_from_euler(0.5 * np.pi, 0, th)
        #   wpose.position.x = grasppose.position.x - radius * np.sin(th)
        #   wpose.position.x = grasppose.position.y - radius + radius* np.sin(th)
        #   wpose.position.z = grasppose.position.z 
        #   wpose.orientation = quaternion
        #   waypoints.append(copy.deepcopy(wpose))
        #   th += 0.04

        # (plan, fraction) = arm.compute_cartesian_path(
        #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        # )  # jump_threshold

        # arm.execute(plan, wait=True)
        


        print("Finished")

if __name__ == "__main__":
    try:
        jointspace_control()
    except rospy.ROSInterruptException:
        pass
