#!/usr/bin/env python
#*-* coding: utf-8 *-*

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
# 
"""
CheYou Ego Vehicle, modified from carla example  Ego-vehicle

"""

import sys
import glob
import os
import random
import math
import json
import datetime
import math
import numpy
import rospy
import tf
#import tf_ros
from tf2_msgs.msg import TFMessage

from carla_ego_vehicle.carla_ego_vehicle import CarlaEgoVehicle
import carla

class CheYou_EgoVehicle(CarlaEgoVehicle):
    """
    CheYou Ego Vehicle
    """
    def __init__(self):
        super(CheYou_EgoVehicle,self).__init__()
        #这个订阅者，每当从话题/tf接收到车辆的定位消息的时候，就将其转换为虚拟场景中仿真车辆位置，并
        #刷新这个仿真车辆在虚拟场景中的位置
        self.teleport_subscriber = rospy.Subscriber(
            "/tf",TFMessage, self.teleport_command_updated)
        
        self.tf_listener = tf.TransformListener()

        #这个变量用来保存ego_vehicle的初始生成位置（特别的，在Z轴方向，是实际的位置，而不是指定的位置）
        self.__InitialPosition = carla.Transform()   
    

    def restart(self):
        super(CheYou_EgoVehicle,self).restart()
        #确保ego_vehicle已经着陆，以便获得正确的Z方向位置
        rospy.sleep(5)
        self.__InitialPosition = self.player.get_transform()

        rospy.loginfo("ego_vehicle initial transform in UE4: x={} y={} z={} yaw={}".format(self.__InitialPosition.location.x,
                                                                  self.__InitialPosition.location.y,
                                                                  self.__InitialPosition.location.z,
                                                                  self.__InitialPosition.rotation.yaw))      
        #禁止物理模拟，防止平移和静止的时候的车辆上下晃动
        self.player.set_simulate_physics(False)  
	    #Disable autopilot
        self.player.set_autopilot(False)
        

    def teleport_command_updated(self,ros_TFMessage):   
        try:
            (position, quaternion) = self.tf_listener.lookupTransform(
                '/map', self.role_name, rospy.Time())
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
            carla_trans = carla.Transform()
            carla_trans.rotation.roll = math.degrees(roll)
            carla_trans.rotation.pitch = math.degrees(pitch)
            carla_trans.rotation.yaw = -math.degrees(yaw)
            carla_trans.location.x = position[0]
            carla_trans.location.y = -position[1]
            carla_trans.location.z = position[2]
            self.player.set_transform(carla_trans) 
            rospy.loginfo("Teleport vehicle to x=%f y=%f z=%f", carla_trans.location.x, carla_trans.location.y, carla_trans.location.z)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x = 0
            y = 0
            z = 0
            yaw = 0      
    
    def run(self):
        """
        main loop
        """
        client = carla.Client(self.host, self.port)
        client.set_timeout(2.0)
        self.world = client.get_world()
        self.restart()

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    ego_vehicle = CheYou_EgoVehicle()
    try:
        ego_vehicle.run()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == '__main__':
    main()
