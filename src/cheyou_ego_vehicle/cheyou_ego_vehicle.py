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
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import geometry_msgs
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
        
        self.__listener = tf.TransformListener()

        #这个变量用来保存ego_vehicle的初始生成位置（特别的，在Z轴方向，是实际的位置，而不是指定的位置）
        self.__InitialPosition = carla.Transform()   
        
        self._rate = rospy.Rate(30.0)	    
        self._br = tf.TransformBroadcaster() 
    

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
	    #Set autopilot to output throttle and steer
        self.player.set_autopilot(False)
        
    #这个函数在ROS的TF树中增加一个新的Frame,新的Frame为UE4地图坐标（右手系），它和map的相对位置和姿态就是小车在UE4地图中的
    #初始位置和姿态
    def add_UE4_transform(self):
        x = self.__InitialPosition.location.x
        z = self.__InitialPosition.location.z
        #将初始值变为右手系
        y = -self.__InitialPosition.location.y

        quat = quaternion_from_euler(
                math.radians(float(self.__InitialPosition.rotation.roll)), 
                math.radians(float(self.__InitialPosition.rotation.pitch)),
                #将初始值变为右手系
                math.radians(-float(self.__InitialPosition.rotation.yaw)) )

        self._br.sendTransform((x,y,z),
                         (quat[0],quat[1],quat[2],quat[3]),
                         rospy.Time.now(),
                         "map",
                         "UE4_map")
	

    def teleport_command_updated(self,ros_TFMessage):   
        try:
            #cartographer发布的/tf消息，我们定义了两个坐标系 map->base_link，base_link是车身坐标系 
            #我们又增加了一个新的坐标系UE4_map,base_link和UE4_map之间的相对关系才是我们需要的
            (trans,rot) = self.__listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("CheYou: tf transform exception!")
            return


        #trans =self.__transformer.lookupTransform('UE4_map','base_link',rospy.Time())
        carla_trans = carla.Transform()
            
        carla_trans.location.x = trans[0] + self.__InitialPosition.location.x
        #UE4使用左手系, Y方向与ROS系统坐标系方向相反, be careful the self.__InitialPosition.location.y is left hand coordinate value
        carla_trans.location.y = - (trans[1]-self.__InitialPosition.location.y)
        #当前我们的环境假定是二维平面，不要更新在Z方向的数据
        carla_trans.location.z =  self.__InitialPosition.location.z
            
        quaternion = (
            rot[0],
            rot[1],
            rot[2],
            rot[3]
        )
        _, _, yaw = euler_from_quaternion(quaternion)
        #UE4使用左手系, 偏航方向与ROS系统的偏航方向正好相反
        carla_trans.rotation.yaw = - math.degrees(yaw)
            
        #当前我们的环境假定是二维平面，不要更新roll和pitch
        carla_trans.rotation.roll = 0.0
        carla_trans.rotation.pitch = 0.0 

          
        if self.player: 
            self.player.set_transform(carla_trans)         
            rospy.loginfo("Teleport vehicle to x=%f y=%f z=%f", carla_trans.location.x, carla_trans.location.y, carla_trans.location.z) 
        
    
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

#        while not rospy.is_shutdown():
#            self.add_UE4_transform()
#            self._rate.sleep()


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
