"""
demonstrate.py

Collect interleaved demonstrations (in the case of kinesthetic teaching) of recording a kinesthetic demo,
then (optionally) playing back the demonstration to collect visual states.

As we're using Polymetis, you should use the following to launch the robot controller:
    > launch_robot.py --config-path /home/iliad/Projects/oncorr/conf/robot --config-name robot_launch.yaml timeout=15;
    > launch_gripper.py --config-path /home/iliad/Projects/oncorr/conf/robot --config-name gripper_launch.yaml;

References:
    - https://github.com/facebookresearch/fairo/blob/main/polymetis/examples/2_impedance_control.py
    - https://github.com/AGI-Labs/franka_control/blob/master/record.py
    - https://github.com/AGI-Labs/franka_control/blob/master/playback.py
"""
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
from tap import Tap
from pynput.keyboard import Key, Listener
import sys
import select
import tty
import termios

from cam.utils import VideoRecorder

from robot.xarm_env import XArmCmSafeEnvironment
from robot.data import RoboDemoDset
from robot.utils import HZ


class ArgumentParser(Tap):
    # fmt: off
    task: str  # Task ID for demonstration collection
    data_dir: Path = Path("data/demos/")  # Path to parent directory for saving demonstrations

    use_gripper: bool = False

    # Task Parameters
    include_visual_states: bool = True  # Whether to run playback/get visual states (only False for now)
    max_time_per_demo: int = 21  # Max time (in seconds) to record demo -- default = 21 seconds

    random_reset: bool = False  # randomly initialize home pose with an offset

    # Collection Parameters
    collection_strategy: str = "kinesthetic"  # How demos are collected :: only `kinesthetic` for now!
    controller: str = "joint"  # Demonstration & playback uses a Joint Impedance controller...
    resume: bool = True  # Resume demonstration collection (on by default)

    plot_trajectory: bool = False  # generate html plot for visualizing trajectory
    show_viewer: bool = False
    # fmt: on


def demonstrate() -> None:
    args = ArgumentParser().parse_args()

    # Make directories for "raw" recorded states, and playback RGB states...
    #   > Note: the "record" + "playback" split is use for "kinesthetic" demos for obtaining visual state w/o humans!
    demo_raw_dir = args.data_dir / args.task / "record-raw"
    os.makedirs(demo_raw_dir, exist_ok=args.resume)
    if args.include_visual_states:
        demo_rgb_dir = args.data_dir / args.task / "playback-rgb"
        os.makedirs(demo_rgb_dir, exist_ok=args.resume)

    # data saving yay
    video_recorder = VideoRecorder(save_dir=demo_rgb_dir, fps=HZ)
    h5py_dset_path = args.data_dir / args.task / "demos.hdf"
    dset = RoboDemoDset(save_path=h5py_dset_path, read_only_if_exists=False)

    # Initialize environment in `record` mode...
    print("[*] Initializing Robot Connection...")
    env = XArmCmSafeEnvironment(
        control_frequency_hz=HZ,
        use_camera=args.include_visual_states,
        use_gripper=args.use_gripper,
        # TODO: create some sort of button pressing mechanism to open and close the gripper,
        random_reset_home_pose=args.random_reset
    )
    # env = XArmEnv(control_frequency_hz=HZ, control_mode='angular',
    #               use_camera=args.include_visual_states,
    #               use_gripper=args.use_gripper,
    #               random_reset_home_pose=args.random_reset)

    # Initializing Button Control... TODO(siddk) -- switch with ASR
    # print("[*] Connecting to Button Controller...")
    # buttons, demo_index = Buttons(), 1

    # If `resume` -- get "latest" index
    if args.resume:
        files = os.listdir(demo_rgb_dir) if args.include_visual_states else os.listdir(demo_raw_dir)
        if len(files) > 0:
            demo_index = max([int(x.split("-")[-1].split(".")[0]) for x in files]) + 1
        else:
            demo_index = 0

    # vc Start Recording Loop
    print("[*] Starting Demo Recording Loop...")
    while True:
        print(f"[*] Starting to Record Demonstration `{demo_index}`...")
        demo_file = f"{args.task}-{datetime.now().strftime('%m-%d')}-{demo_index}.npz"

        # Set `record`

        # Reset environment & wait on user input...
        obs = env.reset()
        print('here')
        # env.set_mode("record")

        print(
            "[*] Ready to record!\n"
            f"\tYou have `{args.max_time_per_demo}` secs to complete the demo, and can use (X) to stop recording.\n"
            "\tPress (Y) to quit, and (A) to start recording!\n "
        )
        # Loop on valid button input...

        console = input()
        # print(console)
        a = console == 'a'
        x = console == 'x'
        y = console == 'y'
        while not a and not y:
            console = input()
            a = console == 'a'
            x = console == 'x'
            y = console == 'y'

        # Quit if (Y)...
        if y:
            break

        # Go, go, go!
        print("\t=>> Started recording... press (X) to terminate recording!")

        # rgbs = []

        def isData():
            return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            ee_poses = []
            # angle_poses = []
            # print(args.max_time_per_demo)
            # print(HZ)
            for _ in range(int(args.max_time_per_demo * HZ) - 1):
                # visualize if camera
                if args.show_viewer:
                    bgr = cv2.cvtColor(obs["rgb_image"], cv2.COLOR_RGB2BGR)
                    cv2.imshow('RGB', bgr)
                    cv2.waitKey(1)

                # Get Button Input (only if True) --> handle extended button press...

                # Terminate...
                if isData():
                    c = sys.stdin.read(1)
                    print(c)
                    if c == 'x':  # x1b is ESC
                        print("\tHit (X) - stopping recording...")
                        break
                # Otherwise no termination, keep on recording...
                else:
                    obs, _, _, _ = env.step(None)
                    ee_poses.append(obs["ee_pos"])
                    # angle_poses.append(obs['q'])
                    # rgbs.append(obs["rgb_image"])
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Drop into Recording Loop --> for `record` mode, we really only care about joint positions
        #   =>> TODO(siddk) - handle Gripper?
        # ee_poses = []
        # print(args.max_time_per_demo)
        # print(HZ)
        # for _ in range(int(args.max_time_per_demo * HZ) - 1):
        #     # visualize if camera
        #     if args.show_viewer:
        #         bgr = cv2.cvtColor(obs["rgb_image"], cv2.COLOR_RGB2BGR)
        #         cv2.imshow('RGB', bgr)
        #         cv2.waitKey(1)

        #     # Get Button Input (only if True) --> handle extended button press...

        #     # console = input()
        #     # print(console)
        #     # a = console == 'a'
        #     # x = console =='x'
        #     # y = console =='y'
        #     # opt = ""
        #     # def on_press(key):
        #     #     if key.char == "x":
        #     #         global opt 
        #     #         opt = key.char
        #     #         return False

        #     # with Listener(on_press=on_press) as listener:
        #     #     listener.join()

        #     console = input()
        #     # print(console)
        #     a = console == 'a'
        #     x = console =='x'
        #     y = console =='y'

        #     # Terminate...
        #     if x or _ == 150:
        #         print("\tHit (X) - stopping recording...")
        #         break
        #     # Otherwise no termination, keep on recording...
        #     else:
        #         obs, _, _, _ = env.step(None)
        #         ee_poses.append(obs["ee_pos"])
        #         # rgbs.append(obs["rgb_image"])
        # Close Environment
        env.close()

        # Save "raw" demonstration...
        np.savez(str(demo_raw_dir / demo_file), hz=HZ, ee_poses=ee_poses)

        # Enter Phase 2 -- Playback (Optional if not `args.include_visual_states`)
        do_playback = True
        if args.include_visual_states:
            print("[*] Entering Playback Mode - Please reset the environment to beginning and get out of the way!")
        else:
            # Loop on valid user button input...
            print("[*] Optional -- would you like to replay demonstration? Press (A) to playback, and (X) to continue!")
            console = input()
            a = console == 'a'
            x = console == 'x'
            y = console == 'y'
            while not a and not x:
                console = input()
                a = console == 'a'
                x = console == 'x'
                y = console == 'y'

            # Skip Playback!
            if x:
                do_playback = False

        # Special Playback Handling -- change gains, and replay!
        jas = []
        eef_poses = []
        rgbs = []
        if do_playback:
            # TODO(siddk) -- handle Camera observation logging...
            obs = env.reset()
            # eef_poses.append(obs["ee_pose"].copy())
            # eef_xyzs.append(obs["ee_pose"][:3].copy())
            # jas.append(obs["q"].copy())
            rgbs.append(obs["rgb_image"].copy())
            # jas.append(obs['q'])
            eef_poses.append(obs['ee_pos'])

            # Block on User Ready -- Robot will move, so this is for safety...
            print("\tReady to playback! Get out of the way, and hit (A) to continue...")
            console = input()
            a = console == 'a'
            x = console == 'x'
            y = console == 'y'
            while not a:
                console = input()
                a = console == 'a'
                x = console == 'x'
                y = console == 'y'

            # Execute Trajectory
            print("\tReplaying...")
            for idx in range(len(ee_poses)):
                obs, _, _, _ = env.step(ee_poses[idx])
                rgbs.append(obs["rgb_image"].copy())
                # jas.append(obs['q'])
                eef_poses.append(obs['ee_pos'])
            # Close Environment
            env.close()

        if args.plot_trajectory:
            from viz.plot import PlotlyScene, plot_transform, plot_points_sequence
            scene = PlotlyScene(
                size=(600, 600), x_range=(-1, 1), y_range=(-1, 1), z_range=(-1, 1)
            )
            plot_transform(scene.figure, np.eye(4), label="world origin")
            # eef_xyzs_np = np.array(eef_xyzs).T # N x 3
            plot_points_sequence(scene.figure, points=eef_xyzs_np)
            scene.plot_scene_to_html("test")

        print(len(rgbs))

        # Move on?
        print("Next? Press (A) to save and continue or (Y) to quit without saving or (X) to retry demo and skip save")

        # Loop on valid button input...        
        console = input()[0]
        a = console == 'a'
        x = console == 'x'
        y = console == 'y'
        while not a and not y and not x:
            console = input()[0]
            a = console == 'a'
            x = console == 'x'
            y = console == 'y'
        # Exit...
        if y:
            break

        # Bump Index
        if not x:
            for frame_idx, rgb_frame in enumerate(rgbs):
                if frame_idx == 0:
                    video_recorder.init(rgb_frame)
                else:
                    video_recorder.record(rgb_frame)
            save_str = str(demo_index).zfill(3)
            video_recorder.save(f"{save_str}.mp4")

            rgb_np = np.expand_dims(np.array(rgbs), axis=0)  # 1, horizon, h, w, c
            ja_np = np.expand_dims(np.array(jas), axis=0)  # 1, horizon, 7
            eefpose_np = np.expand_dims(np.array(eef_poses), axis=0)  # 1, horizon, 7
            print(rgb_np.shape)
            dset.add_traj(rgbs=rgb_np, joint_angles=ja_np, eef_poses=eefpose_np)  # TODO add  joint_angles=ja_np bcak in
            demo_index += 1

    # And... that's all folks!
    print("[*] Done Demonstrating -- Cleaning Up! Thanks 🤖🚀")


if __name__ == "__main__":
    demonstrate()
