#   无人机初始化位置的配置 

class DeploymentPlanner:
    def __init__(self):
        # 这里初始化无人机的初始基站位置  UE中m为单位
        self.station_offsets = {
            "JIZHAN1": [
                (-2.0, -2.0, -3.5),
                (-2.0,  0.0, -3.5),
                (-2.0,  2.0, -3.5),
            ],
            "JIZHAN2": [
                ( 2.0, -3.0, -3.5),
                ( 2.0, -1.0, -3.5),
                ( 2.0,  1.0, -3.5),
                ( 2.0,  3.0, -3.5),
            ]
        }       

    def build_slots(self, world):
        """
        输入:
            world: WorldModel

        输出:
            {
                "Drone1": (x, y, z),
                ...
            }
        """
        drone_slots = {}

        for station_tag, station in world.base_stations.items():
            if station.position_world is None:
                raise RuntimeError(f"{station_tag} position_world is None")
            # 读取基站目标位置
            sx, sy, sz = station.position_world
            # 读取各个无人机自己的偏移位置
            offsets = self.station_offsets[station_tag]
            # 查看动态生成的无人机数量
            uav_list = station.assigned_uavs
            # 比较无人机数量是否一致 
            if len(offsets) != len(uav_list):
                raise RuntimeError(
                    f"{station_tag} 偏移数量({len(offsets)}) "
                    f"与无人机数量({len(uav_list)})不一致"
                )
            # 对各个无人机设置最好的spawn位置  
            for uav_name, (dx, dy, dz) in zip(uav_list, offsets):
                drone_slots[uav_name] = (sx + dx, sy + dy, sz + dz)

        return drone_slots