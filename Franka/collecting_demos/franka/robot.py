''' Robot Server Environment Wrapper'''

# robot imports
from polymetis import RobotInterface, GripperInterface
from real_robot_ik.robot_ik_solver import RobotIKSolver

# utility specific imports
from transformations import euler_to_quat, quat_to_euler
from terminal_utils import run_terminal_command
import torch
import time
import os


class FrankaRobot:
    def __init__(self, control_hz=20):
        self._robot = RobotInterface(ip_address="172.16.0.1")
        self._gripper = GripperInterface(ip_address="172.16.0.1")
        self._ik_solver = RobotIKSolver(self._robot, control_hz=control_hz)
        self._controller_restart = 0

    def launch_robot(self):
        # WIP; Do not use.
        self._robot_process = run_terminal_command('bash ' + os.getcwd() + '/franka/launch_robot.sh')
        self._gripper_process = run_terminal_command('bash ' + os.getcwd() + '/franka/launch_gripper.sh')
        time.sleep(5)

    def kill_robot(self):
        self._robot_process.terminate()
        self._gripper_process.terminate()

    def update_pose_quat(self, pos, quat):
        '''Expect [x,y,z], [yaw, pitch, roll]'''

        desired_pos = torch.Tensor(pos)
        desired_quat = torch.Tensor(quat)

        desired_qpos, _ = self._ik_solver.compute(desired_pos, desired_quat)
        feasible_pos, feasible_quat = self._robot.robot_model.forward_kinematics(desired_qpos)
        feasible_pos, feasible_angle = feasible_pos.numpy(), quat_to_euler(feasible_quat.numpy())

        if not self._robot.is_running_policy():
            self._robot.start_cartesian_impedance()

        try: self._robot.update_desired_joint_positions(desired_qpos)
        except:
            print('impedance controller failed, restarting and skipping this step')
            self._controller_restart += 1
            print(f'controller reset tracker: {self._controller_restart}\n')
            self._robot.start_cartesian_impedance()
            # self._robot.update_desired_joint_positions(desired_qpos)

        return feasible_pos, feasible_angle
    
    def update_pose(self, pos, angle):
        '''Expect [x,y,z], [yaw, pitch, roll]'''

        desired_pos = torch.Tensor(pos)
        desired_quat = torch.Tensor(euler_to_quat(angle))

        desired_qpos, _ = self._ik_solver.compute(desired_pos, desired_quat)
        feasible_pos, feasible_quat = self._robot.robot_model.forward_kinematics(desired_qpos)
        feasible_pos, feasible_angle = feasible_pos.numpy(), quat_to_euler(feasible_quat.numpy())

        if not self._robot.is_running_policy():
            self._robot.start_cartesian_impedance()

        try: self._robot.update_desired_joint_positions(desired_qpos)
        except:
            print('impedance controller failed, restarting and skipping this step')
            self._controller_restart += 1
            print(f'controller reset tracker: {self._controller_restart}\n')
            self._robot.start_cartesian_impedance()
            # self._robot.update_desired_joint_positions(desired_qpos)

        return feasible_pos, feasible_angle

    def update_joints(self, joints):
        joints = torch.Tensor(joints)
        try:
            self._robot.move_to_joint_positions(joints)
        except:
            self._robot.terminate_current_policy()
            time.sleep(2)
            self._robot.move_to_joint_positions(joints)

        self._robot.start_cartesian_impedance()

    def update_gripper(self, flag):
        if flag < 0:
            desired_gripper = 0.00
        elif flag >= 0:
            desired_gripper = 0.085
        self._gripper.goto(width=desired_gripper, speed=0.1, force=1000)

    def get_joint_positions(self):
        return self._robot.get_joint_positions().numpy()

    def get_joint_velocities(self):
        return self._robot.get_joint_velocities().numpy()

    @property
    def gripper_width(self):
        return self._gripper.get_state().width

    def get_gripper_state(self):
        return self._gripper.get_state().width

    def get_ee_pos(self):
        '''Returns [x,y,z]'''
        pos, quat = self._robot.get_ee_pose()
        return pos.numpy()

    def get_ee_angle(self):
        '''Returns [yaw, pitch, roll]'''
        pos, quat = self._robot.get_ee_pose()
        angle = quat_to_euler(quat.numpy())
        return angle
    
    def get_ee_quat(self):
        '''Returns [yaw, pitch, roll]'''
        pos, quat = self._robot.get_ee_pose()        
        return quat.numpy()
