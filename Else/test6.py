import cosysairsim as airsim

VN = "Drone1"
GOAL = "ZAW"   # 这里现在当作“目标Tag”，而不是已知名字

client = airsim.MultirotorClient()
client.confirmConnection()

def ue_log(key, value="", severity=0, also_print=True):
    """
    key: fixed label shown on UE screen (same key will overwrite)
    value: dynamic text
    severity: 0..3 (color)
    """
    if also_print:
        if value != "":
            print(f"{key}: {value}")
        else:
            print(key)
    try:
        client.simPrintLogMessage(str(key), str(value), int(severity))
    except Exception:
        pass

ue_log("STATE", "CONNECTED")

client.enableApiControl(True, vehicle_name=VN)
ue_log("API", "CONTROL=ON")

client.armDisarm(True, vehicle_name=VN)
ue_log("ARM", "ARMED")

ue_log("TASK", "TAKEOFF")
client.takeoffAsync(vehicle_name=VN).join()

ue_log("TASK", "CLIMB Z=-40")
client.moveToZAsync(-40, 4, vehicle_name=VN).join()

ue_log("TASK", f"FIND TAG {GOAL}")

# Find goal by TAG (unknown name initially)
try:
    pairs = client.simListSceneObjectsTags(rf"^{GOAL}$")  # returns [[name, tag], ...]
except Exception as e:
    pairs = []
    ue_log("ERROR", f"simListSceneObjectsTags failed: {e}", severity=2)

ue_log("MATCHES", str(len(pairs)))

if len(pairs) == 0:
    ue_log("RESULT", f"NOT FOUND TAG: {GOAL}", severity=2)
    ue_log("TASK", "LAND")
    client.landAsync(vehicle_name=VN).join()
    client.armDisarm(False, vehicle_name=VN)
    client.enableApiControl(False, vehicle_name=VN)
else:
    real_name = pairs[0][0]  # first match -> name
    pose = client.simGetObjectPose(real_name)
    p = pose.position

    ue_log("GOAL", real_name)  # now we know the real object name
    ue_log("GOAL_POS", f"x={p.x_val:.2f} y={p.y_val:.2f} z={p.z_val:.2f}")

    ue_log("TASK", "GO TO GOAL")
    client.moveToPositionAsync(p.x_val, p.y_val, p.z_val - 5.0, 3.0, vehicle_name=VN).join()

    ue_log("RESULT", "ARRIVED -> HOVER")
    client.hoverAsync(vehicle_name=VN).join()  