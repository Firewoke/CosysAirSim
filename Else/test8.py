import time
import cosysairsim as airsim

client = airsim.MultirotorClient()
client.confirmConnection()


objects = client.simListSceneObjects()
for obj in objects:
    print(obj)

actors = client.simListSceneObjects(r"^Actor_.*")
print("Actor_* count:", len(actors))
print(actors[:20])
vn = "Drone1"
client.enableApiControl(True, vehicle_name=vn)
client.armDisarm(True, vehicle_name=vn)
client.takeoffAsync(vehicle_name=vn).join()

