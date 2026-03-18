import cosysairsim as airsim
import os
import time

# 连接无人机
myDrone = "UAV1"
client = airsim.MultirotorClient()
client.confirmConnection()

# 拿到电机控制权 + API控制
client.armDisarm(True, vehicle_name= myDrone)
client.enableApiControl(True, vehicle_name= myDrone)


