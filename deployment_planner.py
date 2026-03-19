from config import GROUND_DEPLOY_OFFSETS


class DeploymentPlanner:
    def __init__(self, station_offsets=None):
        self.station_offsets = station_offsets or GROUND_DEPLOY_OFFSETS

    def build_slots(self, world):
        drone_slots = {}

        for station_tag, station in world.base_stations.items():
            if station.position_world is None:
                raise RuntimeError(f"{station_tag} position_world is None")

            if station_tag not in self.station_offsets:
                raise RuntimeError(f"{station_tag} 未配置 GROUND_DEPLOY_OFFSETS")

            sx, sy, sz = station.position_world
            offsets = self.station_offsets[station_tag]
            uav_list = station.assigned_uavs

            if len(offsets) != len(uav_list):
                raise RuntimeError(
                    f"{station_tag} 偏移数量({len(offsets)}) "
                    f"与无人机数量({len(uav_list)})不一致"
                )

            for uav_name, (dx, dy, dz) in zip(uav_list, offsets):
                drone_slots[uav_name] = (sx + dx, sy + dy, sz + dz)

        return drone_slots