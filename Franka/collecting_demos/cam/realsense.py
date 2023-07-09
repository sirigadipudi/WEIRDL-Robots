import time
import threading

import numpy as np
import pyrealsense2 as rs


def get_connected_device_sns():
    context = rs.context()
    devices = [d.get_info(rs.camera_info.serial_number) for d in context.devices]
    return devices

def get_connected_device_info():
    context = rs.context()
    devices = [
        (d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number)) for d in context.devices
    ]
    return devices

PREFERRED_SERIAL_NUMBER = "215122255998" # D455
class RealSenseInterface:
    def __init__(self, serial_number: str = None):
        if serial_number is None:
            sns = get_connected_device_sns()
            assert len(sns) != 0
            if PREFERRED_SERIAL_NUMBER in sns:
                self._serial_number = PREFERRED_SERIAL_NUMBER
            else:
                self._serial_number = sns[0]
        else:
            self._serial_number = serial_number

        self._fps = 30

        pipeline, profile, device = RealSenseInterface.build_pipeline(
            self._serial_number, self._fps
        )

        self._pipeline = pipeline
        self._colorizer = rs.colorizer()

        self._depth_intrinsics = (
            profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        )
        self._color_intrinsics = (
            profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        )

        self._align_to_depth = False
        if self._align_to_depth:
            self._aligner = rs.align(rs.stream.depth)
            self._default_intrinsics = self._depth_intrinsics
        else:
            self._aligner = rs.align(rs.stream.color)
            self._default_intrinsics = self._color_intrinsics

        self._K = np.array([
            [self._default_intrinsics.fx, 0, self._default_intrinsics.ppx],
            [0, self._default_intrinsics.fy, self._default_intrinsics.ppy],
            [0, 0, 1],
        ])

        # Threading
        self._img_lock = threading.Lock()
        self._new_frame = False

        self._depth_img = None
        self._color_img = None
        self._colored_depth_img = None

        self._warmup_time = 1.0
        time.sleep(self._warmup_time)

        self._should_run = True
        self._io_thread = threading.Thread(target=self._start)
        self._io_thread.start()

    def _start(self):
        while self._should_run:
            try:
                frames = self._pipeline.wait_for_frames()
                aligned_frames = self._aligner.process(frames)
            except RuntimeError as e:
                print(f"[error][realsense] Could not get frames: {e}")
                time.sleep(1 / self._fps)
                continue

            aligned_depth = aligned_frames.get_depth_frame()
            colored_depth = self._colorizer.colorize(aligned_depth)
            color = aligned_frames.get_color_frame()

            raw_depth_img = np.asanyarray(aligned_depth.get_data())
            # colored_depth_img = np.asanyarray(colored_depth.get_data())
            color_img = np.asanyarray(color.get_data()).astype(np.uint8)

            with self._img_lock:
                self._depth_img = raw_depth_img
                # self._colored_depth_img = colored_depth_img
                self._color_img = color_img
                self._new_frame = True

    def stop(self):
        self._should_run = False
        try:
            while self._io_thread.is_alive():
                self._io_thread.join()
        except RuntimeError as e:
            print(f"[error][realsense] Coult not stop camera: {e}")


    @staticmethod
    def build_pipeline(
        serial_number: str,
        fps: int,
        color_resolution=(640, 480), # width, height
        depth_resolution=(640, 480), # width, height
    ):
        print(f"[realsense] building pipeline for {serial_number} at {fps} fps with color {color_resolution} and depth {depth_resolution}")
        config = rs.config()
        config.enable_device(serial_number)
        config.enable_stream(
            rs.stream.depth,
            depth_resolution[0],
            depth_resolution[1],
            rs.format.z16,
            fps
        )
        config.enable_stream(
            rs.stream.color,
            color_resolution[0],
            color_resolution[1],
            rs.format.rgb8,
            fps
        )

        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        device = profile.get_device()
        print(f"[realsense] done building pipeline for {serial_number}")

        return pipeline, profile, device


    def get_latest_rgbd(self, block_until_new_frame: bool = False):
        rgb, d = None, None

        if block_until_new_frame:
            while not self._new_frame:
                time.sleep(1 / (self._fps * 2))

        while rgb is None and d is None:
            if self._color_img is None or self._depth_img is None:
                continue

            with self._img_lock:
                rgb = np.copy(self._color_img)
                d = np.copy(self._depth_img)
                self._new_frame = False
        return rgb, d

    def get_latest_rgb(self, block_until_new_frame: bool = False):
        img = None

        if block_until_new_frame:
            while not self._new_frame:
                time.sleep(1 / (self._fps * 2))

        while img is None:
            with self._img_lock:
                img = np.copy(self._color_img)
                self._new_frame = False

        return img

    def get_latest_d(self, block_until_new_frame: bool = False):
        img = None

        if block_until_new_frame:
            while not self._new_frame:
                time.sleep(1 / (self._fps * 2))

        while img is None:
            with self._img_lock:
                img = np.copy(self._depth_img)
                self._new_frame = False

        return img

    def get_camera_k(self):
        return self._K



if __name__ == "__main__":
    import cv2

    # print all cameras
    cams = get_connected_device_info()
    print(f"Currently connected cameras:")
    for idx, cam in enumerate(cams):
        print(f"[{idx}]: {cam}")
    res = input("which cam?")
    if res == "n":
        exit()
    elif res == "default":
        serial_number = "215122255998"
    else:
        cam_idx = int(res)
        serial_number = cams[cam_idx][1]

    # start camera feed
    rsi = RealSenseInterface(serial_number=serial_number)
    init = True

    while True:
        rgb, d = rsi.get_latest_rgbd()

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if init:
            print(bgr.shape)
            print(bgr.dtype)
            init = False

        cv2.imshow('RGB', bgr)
        cv2.imshow('Depth', d)

        # Wait for key to shift state
        k = cv2.waitKey(1)
        if k & 0xFF == ord('b'):
            break
        else:
            pass

    rsi.stop()
    cv2.destroyAllWindows()
