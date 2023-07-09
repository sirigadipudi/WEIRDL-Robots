from flask import Flask, jsonify, request, send_file, make_response
from PIL import Image
import numpy as np
import io

app = Flask(__name__)

### KILL SERVER ###
@app.route('/kill_server', methods=['POST'])
def kill_robot_request():
    robot_controller.kill_server()
    return 'Killed Server'

### ROBOT POSE UPDATES ###
@app.route('/update_pose', methods=['POST'])
def update_pose_request():
    pose = np.array(request.json['pose'])
    pos, angle = pose[:3], pose[3:]
    feasible_pos, feasbile_angle = robot_controller.update_pose(pos, angle)
    return jsonify({"feasible_pos": np.array(feasible_pos).tolist(),
                    "feasible_angle": np.array(feasbile_angle).tolist()})

@app.route('/move_to_ee_pose', methods=['POST'])
def move_to_ee_pose_request():
    pose = np.array(request.json['pose'])
    pos = pose[:3]
    robot_controller._robot.move_to_ee_pose(position=pos, time_to_go = 2)
    return 'Pose Updated'

@app.route('/update_joints', methods=['POST'])
def update_joints_request():
    joints = np.array(request.json['joints'])
    robot_controller.update_joints(joints)
    return 'Pose Updated'

@app.route('/update_gripper', methods=['POST'])
def update_gripper_request():
    close_percentage = np.array(request.json['gripper'])
    robot_controller.update_gripper(close_percentage)
    return 'Pose Updated'

### ROBOT STATE REQUESTS ###
@app.route('/get_pos', methods=['POST'])
def get_ee_pos_request():
    robot_pos = robot_controller.get_ee_pos()
    return jsonify({"ee_pos": np.array(robot_pos).tolist()})

@app.route('/get_angle', methods=['POST'])
def get_ee_angle_request():
    robot_angle = robot_controller.get_ee_angle()
    return jsonify({"ee_angle": np.array(robot_angle).tolist()})

@app.route('/get_qpos', methods=['POST'])
def get_qpos_request():
    robot_qpos = robot_controller.get_joint_positions()
    return jsonify({"qpos": np.array(robot_qpos).tolist()})

@app.route('/get_qvel', methods=['POST'])
def get_qvel_request():
    robot_qvel = robot_controller.get_joint_velocities()
    return jsonify({"qvel": np.array(robot_qvel).tolist()})

@app.route('/get_gripper_state', methods=['POST'])
def get_gripper_state_request():
    robot_gripper_state = robot_controller.get_gripper_state()
    return jsonify({"gripper_state": np.array(robot_gripper_state).tolist()})

# # ### IMAGE REQUESTS ###
# @app.route('/read_cameras', methods=['POST'])
# def read_cameras():
#     camera_feed = camera_reader.read_cameras()
#     buffer = io.BytesIO()
    
#     np.savez_compressed(buffer, camera_feed)
#     buffer.seek(0)

#     return send_file(buffer, 'camera_feed.npz')

### SERVER LAUNCHER ###
def start_server(robot, camera=None):
    global robot_controller
    # global camera_reader
    robot_controller = robot
    # camera_reader = camera
    app.run(host='0.0.0.0')


if __name__ == '__main__':
    app.run(host='0.0.0.0')