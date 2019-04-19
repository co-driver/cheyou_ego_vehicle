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
import rospy

from carla_ego_vehicle.carla_ego_vehicle import CarlaEgoVehicle

#注意是如何使用ROS的geometry_msg/Transform 消息.导入carla的Transform，
#为了和ROS的Transform区别，使用as语法，重新命名
#from geometry_msgs.msg import Transform as ROS_Transform
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
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
            "/tf",
            TFMessage, self.teleport_command_updated)
        
        self.__tfBuffer = tf2_ros.Buffer()
        self.__listener = tf2_ros.TransformListener(self.__tfBuffer)
        self.__vehicle_transform = carla.Transform()

        #这个变量用来保存ego_vehicle的初始生成位置（特别的，在Z轴方向，是实际的位置，而不是指定的位置）
        self.__InitialPosition = carla.Transform()        

    def sensors(self):
        """
        define all sensors attached to your ego vehicle
        """
        return [
            {
                'type': 'sensor.camera.rgb',
                'role_name': 'front',
                'x': 2.0, 'y': 0.0, 'z': 2.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.camera.rgb',
                'role_name': 'view',
                'x': -4.5, 'y': 0.0, 'z': 2.8, 'roll': 0.0, 'pitch': -20.0, 'yaw': 0.0,
                'width': 800,
                'height': 600,
                'fov': 100
            },
            {
                'type': 'sensor.lidar.ray_cast',
                'role_name': 'lidar1',
                'x': 0.0, 'y': 0.0, 'z': 2.4, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'range': 5000,
                'channels': 32,
                'points_per_second': 320000,
                'upper_fov': 2.0,
                'lower_fov': -26.8,
                'rotation_frequency': 20
            },
            {
                'type': 'sensor.other.gnss',
                'role_name': 'gnss1',
                'x': 1.0, 'y': 0.0, 'z': 2.0
            },
            {
                'type': 'sensor.other.collision',
                'role_name': 'collision1',
                'x': 0.0, 'y': 0.0, 'z': 0.0
            },
            {
                'type': 'sensor.other.lane_invasion',
                'role_name': 'laneinvasion1',
                'x': 0.0, 'y': 0.0, 'z': 0.0
            }
        ]

    def restart(self):
        super(CheYou_EgoVehicle,self).restart()
        #确保ego_vehicle已经着陆，以便获得正确的Z方向位置
        rospy.sleep(5)
        self.__InitialPosition = self.player.get_transform()
        #高度方向增加2cm,防止移动的时候碰撞
        self.__InitialPosition.location.z += 0.02
        rospy.loginfo("ego_vehicle initial transform: x={} y={} z={} yaw={}".format(self.__InitialPosition.location.x,
                                                                  self.__InitialPosition.location.y,
                                                                  self.__InitialPosition.location.z,
                                                                  self.__InitialPosition.rotation.yaw))        

    def teleport_command_updated(self,ros_TFMessage):   
        try:
            #cartographer发布的/tf消息，我们定义了两个坐标系 map->base_link，base_link是车身坐标系 
            trans =self.__tfBuffer.lookup_transform('map','base_link',rospy.Time())
            
            self.__vehicle_transform.location.x = self.__InitialPosition.location.x + trans.transform.translation.x
            #UE4使用左手系, Y方向与ROS系统坐标系方向相反
            self.__vehicle_transform.location.y = self.__InitialPosition.location.y - trans.transform.translation.y
            #当前我们的环境假定是二维平面，不要更新在Z方向的数据,以及roll和pitch
            self.__vehicle_transform.location.z =  self.__InitialPosition.location.z
            self.__vehicle_transform.rotation.roll = 0.0
            self.__vehicle_transform.rotation.pitch = 0.0

            quaternion = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            _, _, yaw = euler_from_quaternion(quaternion)
            #UE4使用左手系, 偏航方向与ROS系统的偏航方向正好相反
            self.__vehicle_transform.rotation.yaw = self.__InitialPosition.rotation.yaw - math.degrees(yaw)
            if self.player: 
                self.player.set_transform(self.__vehicle_transform)         
                rospy.loginfo("Teleport vehicle to x=%f y=%f z=%f", self.__vehicle_transform.location.x, self.__vehicle_transform.location.y, self.__vehicle_transform.location.z) 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("CheYou: tf2_ros transform exception!")

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
