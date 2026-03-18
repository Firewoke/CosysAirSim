import cosysairsim as airsim
import cv2
import pprint

client = airsim.VehicleClient()
client.confirmConnection()
print("Connected!")

camera_name = "0"
image_type = airsim.ImageType.Scene

# 检测半径（cm）= 200m
client.simSetDetectionFilterRadius(camera_name, image_type, 200 * 100)
# 只检测名字匹配 Cylinder* 的物体
client.simAddDetectionFilterMeshName(camera_name, image_type, "Cylinder*")

while True:
    raw = client.simGetImage(camera_name, image_type)
    if not raw:
        continue

    img = cv2.imdecode(airsim.string_to_uint8_array(raw), cv2.IMREAD_UNCHANGED)

    dets = client.simGetDetections(camera_name, image_type)
    if dets:
        for d in dets:
            print("Detected:", pprint.pformat(d))
            x1 = int(d.box2D.min.x_val)
            y1 = int(d.box2D.min.y_val)
            x2 = int(d.box2D.max.x_val)
            y2 = int(d.box2D.max.y_val)
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(img, d.name, (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 1)

    cv2.imshow("AirSim Detection", img)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
    elif k == ord('c'):
        client.simClearDetectionMeshNames(camera_name, image_type)
        print("Cleared detection filters")
    elif k == ord('a'):
        client.simAddDetectionFilterMeshName(camera_name, image_type, "Cylinder*")
        print("Added Cylinder* filter")

cv2.destroyAllWindows()