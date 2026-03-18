import os
import time
import threading
import cv2
import numpy as np
import cosysairsim as airsim


def record_vehicle_camera_to_mp4(
    client,
    vehicle_name: str,
    camera_name: str = "0",
    out_path: str = "D:/recordings/drone_scene.mp4",
    fps: float = 10.0,
    duration_sec: float = 30.0,
    stop_event=None,
):
    """
    持续抓取某架无人机某个相机的 Scene 图像，并保存为 mp4。
    可通过 stop_event 提前终止。
    """
    dir_name = os.path.dirname(out_path)
    if dir_name:
        os.makedirs(dir_name, exist_ok=True)

    if fps <= 0:
        raise ValueError("fps must be > 0")

    frame_interval = 1.0 / fps
    writer = None
    start = time.time()
    next_t = start

    try:
        while True:
            # 外部停止
            if stop_event is not None and stop_event.is_set():
                print(f"[REC] stop_event received: {vehicle_name}")
                break

            # 到最大时长
            if time.time() - start >= duration_sec:
                print(f"[REC] duration reached: {vehicle_name}")
                break

            # 控制采样节奏
            now = time.time()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += frame_interval

            # 抓图
            resp = client.simGetImages(
                [airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)],
                vehicle_name=vehicle_name
            )[0]

            if resp.width == 0 or resp.height == 0:
                print(f"[WARN] empty frame from {vehicle_name}:{camera_name}")
                continue

            img_1d = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
            expected_size = resp.height * resp.width * 3
            if img_1d.size != expected_size:
                print(
                    f"[WARN] bad frame size from {vehicle_name}:{camera_name}, "
                    f"got={img_1d.size}, expected={expected_size}"
                )
                continue

            frame = img_1d.reshape(resp.height, resp.width, 3)

            # AirSim/Cosys 通常返回 RGB，OpenCV 写视频用 BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            if writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(out_path, fourcc, fps, (resp.width, resp.height))
                if not writer.isOpened():
                    raise RuntimeError(f"VideoWriter open failed: {out_path}")

            writer.write(frame)

    except Exception as e:
        print(f"[REC ERROR] {vehicle_name}: {e}")

    finally:
        if writer is not None:
            writer.release()
        print(f"[OK] saved video: {out_path}")


def record_vehicle_camera_background(
    vehicle_name: str,
    camera_name: str = "0",
    out_path: str = "D:/recordings/task_full.mp4",
    fps: float = 10.0,
    duration_sec: float = 300.0,
    stop_event=None,
):
    """
    后台线程入口：
    为该录像线程单独创建一个 AirSim client，避免和主控 client 混用。
    """
    rec_client = airsim.MultirotorClient()
    rec_client.confirmConnection()

    record_vehicle_camera_to_mp4(
        rec_client,
        vehicle_name=vehicle_name,
        camera_name=camera_name,
        out_path=out_path,
        fps=fps,
        duration_sec=duration_sec,
        stop_event=stop_event,
    )


def start_vehicle_camera_recording(
    vehicle_name: str,
    camera_name: str = "0",
    out_path: str = "D:/recordings/task_full.mp4",
    fps: float = 10.0,
    duration_sec: float = 300.0,
):
    """
    启动单架无人机的后台录像线程。
    返回:
        thread, stop_event
    """
    stop_event = threading.Event()

    thread = threading.Thread(
        target=record_vehicle_camera_background,
        kwargs={
            "vehicle_name": vehicle_name,
            "camera_name": camera_name,
            "out_path": out_path,
            "fps": fps,
            "duration_sec": duration_sec,
            "stop_event": stop_event,
        },
        daemon=True,
    )
    thread.start()

    print(f"[REC START] vehicle={vehicle_name}, camera={camera_name}, out={out_path}")
    return thread, stop_event


def stop_vehicle_camera_recording(thread, stop_event, join_timeout: float = 5.0):
    """
    停止单个录像线程。
    """
    if stop_event is not None:
        stop_event.set()

    if thread is not None and thread.is_alive():
        thread.join(timeout=join_timeout)

    print("[REC STOP] recording thread stopped.")


def start_multi_vehicle_recording(
    vehicle_names,
    camera_name: str = "0",
    out_dir: str = "D:/recordings",
    fps: float = 10.0,
    duration_sec: float = 300.0,
    file_prefix: str = "",
):
    """
    批量启动多架无人机录像。
    返回:
        recorders = {
            uav_name: {
                "thread": ...,
                "stop_event": ...,
                "out_path": ...
            }
        }
    """
    os.makedirs(out_dir, exist_ok=True)

    recorders = {}
    for uav_name in vehicle_names:
        filename = f"{file_prefix}{uav_name}.mp4" if file_prefix else f"{uav_name}.mp4"
        out_path = os.path.join(out_dir, filename)

        thread, stop_event = start_vehicle_camera_recording(
            vehicle_name=uav_name,
            camera_name=camera_name,
            out_path=out_path,
            fps=fps,
            duration_sec=duration_sec,
        )

        recorders[uav_name] = {
            "thread": thread,
            "stop_event": stop_event,
            "out_path": out_path,
        }

    return recorders


def stop_multi_vehicle_recording(recorders, join_timeout: float = 5.0):
    """
    批量停止多架无人机录像。
    """
    if not recorders:
        return

    for uav_name, rec in recorders.items():
        print(f"[REC STOP REQ] {uav_name}")
        stop_vehicle_camera_recording(
            rec.get("thread"),
            rec.get("stop_event"),
            join_timeout=join_timeout,
        )