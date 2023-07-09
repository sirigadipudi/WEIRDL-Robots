"""
Gym interface to interact with the real robot
"""
import numpy as np 
import time
import gym 

from transformations import add_angles, angle_diff
from camera_utils.multi_camera_wrapper import MultiCameraWrapper
from camera_utils.realsense_camera import gather_realsense_cameras
from server.robot_interface import RobotInterface
from gym.spaces import Box, Dict
    
class LowDimRobotEnv(gym.Env):
    def __init__(
        self,
        # control frequency
        hz=10,
        DoF=3,
        # randomize arm position on reset  
        randomize_ee_on_reset=False,
        # allows user to pause to reset reset of the environment
        pause_after_reset=False,
        # observation space configuration
        hand_centric_view=True, 
        third_person_view=True,
        qpos=True,
        ee_pos=True,
        # pass IP if not running on NUC
        ip_address=None,
        # for state only experiments
        goal_state=None,
        # specify path length if resetting after a fixed length
        max_path_length=None,
        local_cameras=True,
        controller = "cartesian",
    ):

        # gym.Env.__init__(self)
        super().__init__()

        # physics
        self.use_desired_pose = False
        self.max_lin_vel = 0.45 # 0.2 # 0.1
        self.max_rot_vel = 0.4 # 2.0 # 0.5
        self.DoF = DoF
        self.hz = hz

        self._episode_count = 0
        self._max_path_length = max_path_length
        self._curr_path_length = 0
        self.controller = controller
        self._use_local_cameras = local_cameras
        # self.robotif = RobotInterface() 

        # reward config, relevant only for state only experiments
        # self._goal_state = None
        # if goal_state == 'left_open':
        #     self._goal_state = [1, -1, 1, 1]
        # elif goal_state == 'right_closed':
        #     self._goal_state = [1, 1, 1, -1]

        self._goal_state = np.array([ 0.5930528 , 0.29561332,  0.15176179])

        # resetting configuration
        self._peg_insert = True
        self._randomize_ee_on_reset = randomize_ee_on_reset
        self._pause_after_reset = pause_after_reset
        self._gripper_angle = 0.1 if self._peg_insert else 1.544
        self._reset_joint_qpos = np.array([0, 0.423, 0, -1.944, 0.013, 2.219, self._gripper_angle])
        # self._reset_joint_qpos = np.array([0.3077,  1.0451,  0.0956, -1.1400, -0.0037,  2.1512, self._gripper_angle])

        # observation space config
        self._first_person = hand_centric_view
        self._third_person = third_person_view
        self._qpos = qpos
        self._ee_pos = ee_pos

        # action space
        self.action_space = Box(
            np.array([-1] * (self.DoF + 1)), # dx_low, dy_low, dz_low, dgripper_low
            np.array([ 1] * (self.DoF + 1)), # dx_high, dy_high, dz_high, dgripper_high
        )
        # EE position (x, y, z) + gripper width
        if self.DoF == 3:
            self.ee_space = Box(
                np.array([0.38, -0.25, 0.25, 0.00]),
                np.array([0.70, 0.28, 0.35, 0.085]),
            )
        elif self.DoF == 4:
            # EE position (x, y, z) + gripper width
            self.ee_space = Box(
                np.array([0.55, -0.06, 0.15, -1.57, 0.00]),
                np.array([0.73, 0.28, 0.35, 0.0, 0.085]),
            )

        # joint limits + gripper
        self._jointmin = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, 0.0045], dtype=np.float32)
        self._jointmax = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 0.085], dtype=np.float32)
        # joint space + gripper
        self.qpos_space = Box(
            self._jointmin,
            self._jointmax
        )

        # final observation space configuration
        env_obs_spaces = {
            'hand_img_obs': Box(0, 255, (100, 100, 3), np.uint8),
            'third_person_img_obs': Box(0, 255, (100, 100, 3), np.uint8),
            'lowdim_ee': self.ee_space,
            'lowdim_qpos': self.qpos_space,
        }
        if not self._first_person:
            env_obs_spaces.pop('hand_img_obs', None)
        if not self._third_person:
            env_obs_spaces.pop('third_person_img_obs', None)
        if not self._qpos:
            env_obs_spaces.pop('lowdim_qpos', None)
        if not self._ee_pos:
            env_obs_spaces.pop('lowdim_ee', None)
        self.observation_space = Dict(env_obs_spaces)
        print(f'configured observation space: {self.observation_space}')

        # robot configuration
        if ip_address is None:
            from franka.robot import FrankaRobot
            self._robot = FrankaRobot(control_hz=self.hz)
        else:
            self._robot = RobotInterface(ip_address=ip_address)

        self._use_local_cameras = True
        cameras = gather_realsense_cameras()
        self._camera_reader = MultiCameraWrapper(specific_cameras=cameras)

        self._hook_safety = False
        self._bowl_safety = False
        self._safety = self._hook_safety or self._bowl_safety

        # cameras
        self.cam_ext_mat = None
        self.cam_int_mat = None

    def step(self, action):
        start_time = time.time()

        assert len(action) == (self.DoF + 1)
        assert (action.max() <= 1) and (action.min() >= -1)

        if action is not None:
            if self.controller == "cartesian":
                pos_action, angle_action, gripper = self._format_action(action)
                lin_vel, rot_vel = self._limit_velocity(pos_action, angle_action)
                desired_pos, gripper = self._get_valid_pos_and_gripper(self._curr_pos + lin_vel, gripper)
                desired_angle = add_angles(rot_vel, self._curr_angle)
                if self.DoF == 4:
                    desired_angle[2] = desired_angle[2].clip(self.ee_space.low[3], self.ee_space.high[3])
                self._update_robot(desired_pos, desired_angle, gripper)
            else:
                raise NotImplementedError(f"Controller mode `{self.controller}` not supported!")

        comp_time = time.time() - start_time
        sleep_left = max(0, (1 / self.hz) - comp_time)
        time.sleep(sleep_left)
        obs = self.get_observation()

        # reward defaults to 0., unless a goal is specified
        reward = -np.linalg.norm(obs['lowdim_ee'][:3] - self._goal_state[:3])
        self._curr_path_length += 1
        done = False
        if self._max_path_length is not None and self._curr_path_length >= self._max_path_length:
            done = True
        # info = {
        #     'rgb_aligned': obs['hand_img_obs_aligned'], 
        #     'depth': obs['third_person_img_obs'], 
        #     'lowdim_ee': obs['lowdim_ee'],
        # }
        info = 0
        return obs, reward, done, info

    def get_observation(self):
        # get state and images
        current_state = self.get_state()
        current_images = self.get_images()

        # set images
        obs_first = current_images[0]['color_image']
        aligned_obs_first = current_images[0]['shape'] #['shape'] not ['color_aligned']
        obs_third = current_images[1]['depth_aligned']

        # set gripper width
        gripper_width = current_state['current_pose'][-1:]
        # compute and normalize ee/qpos state
        if self.DoF == 3:
            ee_pos = np.concatenate([current_state['current_pose'][:3], gripper_width])
        elif self.DoF == 4:
            ee_pos = np.concatenate([current_state['current_pose'][:3], current_state['current_pose'][5:6], gripper_width])
        qpos = np.concatenate([current_state['joint_positions'],  gripper_width])

        obs_dict = {
            'hand_img_obs': obs_first,
            'third_person_img_obs': obs_third,
            'lowdim_ee': ee_pos,
            'lowdim_qpos': qpos,
        }

        if not self._first_person:
            obs_dict.pop('hand_img_obs', None)
        if not self._third_person:
            obs_dict.pop('third_person_img_obs', None)
        if not self._qpos:
            obs_dict.pop('lowdim_qpos', None)
        if not self._ee_pos:
            obs_dict.pop('lowdim_ee', None)

        return obs_dict
    
    def reset_gripper(self):
        self._robot.update_gripper(-1)

    def reset(self):
        # to move safely, always move up to the highest positions before resetting joints
        if self._safety and self._episode_count > 0:
            cur_pos = self.normalize_ee_obs(np.concatenate([self._curr_pos, [0.]]))[:3]

            if self._hook_safety:
                # if inside the hook, push it back
                while -0.04 <= cur_pos[0] <= 0.3 and 0.66 <= cur_pos[1] <= 0.75 and 0.78 <= cur_pos[2] <= 0.98:
                    self.step(np.array([-0.2, 0., 0., -1.]))
                    cur_pos = self.normalize_ee_obs(np.concatenate([self._curr_pos, [0.]]))[:3]

                # push to the left, till its clear of the hook
                while cur_pos[1] >= 0.10:
                    self.step(np.array([0.0, -0.2, 0., 1.]))
                    cur_pos = self.normalize_ee_obs(np.concatenate([self._curr_pos, [0.]]))[:3]

            if self._bowl_safety:
                # raise the gripper and then reset it
                while cur_pos[2] <= 0.5:
                    self.step(np.array([0.0, 0.0, 1.0, 1.]))
                    cur_pos = self.normalize_ee_obs(np.concatenate([self._curr_pos, [0.]]))[:3]

        self.reset_gripper()
        for _ in range(5):
            self._robot.update_joints(np.array([ 0.09358515,  0.14613883,  0.0718212 , -2.25930071,  0.03704859,
                    2.44177556,  0.3072933 ])) #//was this
            # self._robot.update_joints(self._reset_joint_qpos)
            # Push positions
            # self._robot.update_joints(np.array([ 0.2221,  0.6950,  0.2080, -1.4889, -0.3319,  2.2121,  1.3278]))
            # time.sleep(1)
            # self._robot.update_joints(np.array([ 0.2072,  0.8603,  0.1909, -1.5307, -0.3319,  2.4080,  1.3061]))
            # break
            # self._robot.update_joints(np.array([0.4295,  0.1502, -0.3973, -2.2330, -0.1980,  2.4875,  1.0871]))
            time.sleep(0.5)
            break

        # fix default angle at first joint reset
        if self._episode_count == 0:                              
            self._default_angle = self._robot.get_ee_angle()

        if self._randomize_ee_on_reset:
            desired_position = np.random.uniform(self.ee_space.low, self.ee_space.high)
            self._robot.move_to_ee_pose(desired_position)
            # self._desired_pose = {'position': self._robot.get_ee_pos(),
            #                       'angle': self._robot.get_ee_angle(),
            #                       'gripper': 1}
            # self._randomize_reset_pos()
            time.sleep(0.5)

        if self._pause_after_reset:
            user_input = input("Enter (s) to wait 5 seconds & anything else to continue: ")
            if user_input in ['s', 'S']:
                time.sleep(5)

        # initialize desired pose correctly for env.step
        self._desired_pose = {'position': self._robot.get_ee_pos(),
                              'angle': self._robot.get_ee_angle(),
                              'gripper': 1}

        self._curr_path_length = 0
        self._episode_count += 1

        return self.get_observation()
    
    def normalize_ee_obs(self, obs):
        """Normalizes low-dim obs between [-1,1]."""
        # x_new = 2 * (x - min(x)) / (max(x) - min(x)) - 1
        # x = (x_new + 1) * (max (x) - min(x)) / 2 + min(x)
        # Source: https://stats.stackexchange.com/questions/178626/how-to-normalize-data-between-1-and-1
        normalized_obs = 2 * (obs - self.ee_space.low) / (self.ee_space.high - self.ee_space.low) - 1
        return normalized_obs
    
    def unnormalize_ee_obs(self, obs):
        return (obs + 1) * (self.ee_space.high - self.ee_space.low) / 2 + self.ee_space.low

    def normalize_qpos(self, qpos):
        """Normalizes qpos between [-1,1]."""
        # The ranges for the joint limits are taken from
        # the franka emika page: https://frankaemika.github.io/docs/control_parameters.html
        norm_qpos = 2 * (qpos - self.qpos_space.low) / (self.qpos_space.high - self.qpos_space.low) - 1
        return norm_qpos

    def _format_action(self, action):
        '''Returns [x,y,z], [yaw, pitch, roll], close_gripper'''
        default_delta_angle = angle_diff(self._default_angle, self._curr_angle)
        if self.DoF == 3:
            delta_pos, delta_angle, gripper = action[:-1], default_delta_angle, action[-1]
        elif self.DoF == 4:
            delta_pos, delta_angle, gripper = action[:3], [default_delta_angle[0], default_delta_angle[1], action[3]], action[-1]
        elif self.DoF == 6:
            delta_pos, delta_angle, gripper = action[:3], action[3:6], action[-1]
        return np.array(delta_pos), np.array(delta_angle), gripper

    def _limit_velocity(self, lin_vel, rot_vel):
        """Scales down the linear and angular magnitudes of the action"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        if lin_vel_norm > 1:
            lin_vel = lin_vel / lin_vel_norm
        if rot_vel_norm > 1:
            rot_vel = rot_vel / rot_vel_norm
        lin_vel, rot_vel = lin_vel * self.max_lin_vel / self.hz, rot_vel * self.max_rot_vel / self.hz
        return lin_vel, rot_vel

    def _get_valid_pos_and_gripper(self, pos, gripper):
        '''To avoid situations where robot can break the object / burn out joints,
        allowing us to specify (x, y, z, gripper) where the robot cannot enter. Gripper is included
        because (x, y, z) is different when gripper is open/closed.

        There are two ways to do this: (a) reject action and maintain current pose or (b) project back
        to valid space. Rejecting action works, but it might get stuck inside the box if no action can
        take it outside. Projection is a hard problem, as it is a non-convex set :(, but we can follow
        some rough heuristics.'''

        # clip commanded position to satisfy box constraints
        x_low, y_low, z_low = self.ee_space.low[:3]
        x_high, y_high, z_high = self.ee_space.high[:3]
        pos[0] = pos[0].clip(x_low, x_high) # new x
        pos[1] = pos[1].clip(y_low, y_high) # new y
        pos[2] = pos[2].clip(z_low, z_high) # new z

        '''Prevents robot from entering unsafe territory.

        Unsafe cube is specified as a set of constraints:
        [(x_low, y_low, z_low), (x_high, y_high, z_high)]. Whenever, (x, y, z) falls into 
        any of the boxes, constrained is violated. When specifying one-sided constraint,
        use 2.5/-2.5 as the other limit.

        Assumption: you can only violate _exactly_ one constraint at a time.'''
        if self._safety:
            MAX_LIM = 2.5
            # constraints are computed for normalized observation
            cur_pos = self.normalize_ee_obs(np.concatenate([pos, [0.]]))[:3]
            assert np.linalg.norm(pos - self.unnormalize_ee_obs(np.concatenate([cur_pos, [0.]]))[:3] <= 1e-6) 

            unsafe_box = []

            if self._hook_safety:
                # DO NOT open gripper if passing through the hook
                if 0.08 <= cur_pos[0] <= 0.3 and 0.66 <= cur_pos[1] <= 0.75 and 0.78 <= cur_pos[2] <= 0.98:
                    gripper = -1

                # open gripper
                if gripper >= -0.8:
                    # when height is high enough
                    unsafe_box.append(np.array([[-0.22, 0.14, 0.02], [0.36, MAX_LIM, 0.96]]))
                    # wrist camera hits the hook
                    unsafe_box.append(np.array([[-0.66, 0.14, -MAX_LIM], [0.58, MAX_LIM, 0.02]]))
                # gripper closed
                else:
                    # tunnel for cloth draping
                    unsafe_box.append(np.array([[-0.22, 0.14, 0.78], [0.36, 0.66, MAX_LIM]]))
                    unsafe_box.append(np.array([[-0.22, 0.75, 0.78], [0.36, MAX_LIM, MAX_LIM]]))
                    # height is high enough not to risk wrist camera hitting the hook, so can come closer
                    unsafe_box.append(np.array([[-0.22, 0.14, 0.05], [0.36, MAX_LIM, 0.78]]))
                    # wrist camera hits the hook
                    unsafe_box.append(np.array([[-0.66, 0.14, -MAX_LIM], [0.58, MAX_LIM, 0.05]]))

            elif self._bowl_safety and False:
                width_out = 0.04
                width = 0.04
                # approximate outside dimensions
                y1_out, y2_out = -0.64, 0.69
                x1_out, x2_out = -0.45, 0.66
                z_out = 0.21
                z_out_g = 0.45
                # approximate inside dimensions, y varies based on whether gripper is closed or open
                y1_in_g, y2_in_g = -0.20, 0.25
                y1_in, y2_in = -0.06, 0.12
                x1_in, x2_in = -0.06, 0.42
                z_in = -0.35
                z_in_g = -0.4

                if gripper >= 0.8:
                    # outside, y-axis boxes
                    unsafe_box.append(np.array([[x1_out, y1_out - width_out, -MAX_LIM], [x2_out, y1_out, z_out_g]]))
                    unsafe_box.append(np.array([[x1_out, y2_out, -MAX_LIM], [x2_out, y2_out + width_out, z_out_g]]))
                    # outside, x-axis boxes
                    unsafe_box.append(np.array([[x1_out - width_out, y1_out, -MAX_LIM], [x1_out, y2_out, z_out_g]]))
                    unsafe_box.append(np.array([[x2_out, y1_out, -MAX_LIM], [x2_out + width_out, y2_out, z_out_g]]))
                    # inside, y-axis boxes
                    unsafe_box.append(np.array([[x1_in, y1_in_g, -MAX_LIM], [x2_in, y1_in_g + width, z_in_g]]))
                    unsafe_box.append(np.array([[x1_in, y2_in_g - width, -MAX_LIM], [x2_in, y2_in_g, z_in_g]]))
                    # inside, x-axis boxes
                    unsafe_box.append(np.array([[x1_in, y1_in_g, -MAX_LIM], [x1_in + width, y2_in_g, z_in_g]]))
                    unsafe_box.append(np.array([[x2_in - width, y1_in_g, -MAX_LIM], [x2_in, y2_in_g, z_in_g]]))
                else:
                    # outside, y-axis boxes
                    unsafe_box.append(np.array([[x1_out, y1_out - width_out, -MAX_LIM], [x2_out, y1_out, z_out]]))
                    unsafe_box.append(np.array([[x1_out, y2_out, -MAX_LIM], [x2_out, y2_out + width_out, z_out]]))
                    # outside, x-axis boxes
                    unsafe_box.append(np.array([[x1_out - width_out, y1_out, -MAX_LIM], [x1_out, y2_out, z_out]]))
                    unsafe_box.append(np.array([[x2_out, y1_out, -MAX_LIM], [x2_out + width_out, y2_out, z_out]]))
                    # inside, y-axis boxes
                    unsafe_box.append(np.array([[x1_in, y1_in, -MAX_LIM], [x2_in, y1_in + width, z_in]]))
                    unsafe_box.append(np.array([[x1_in, y2_in - width, -MAX_LIM], [x2_in, y2_in, z_in]]))
                    # inside, x-axis boxes
                    unsafe_box.append(np.array([[x1_in, y1_in, -MAX_LIM], [x1_in + width, y2_in, z_in]]))
                    unsafe_box.append(np.array([[x2_in - width, y1_in, -MAX_LIM], [x2_in, y2_in, z_in]]))

            def _violate(pos):
                for _, constraint in enumerate(unsafe_box):
                    if np.min(np.concatenate([pos - constraint[0], constraint[1] - pos])) >= 0.:
                        return True
                return False

            '''It is possible that it is still violating the constraint after the first projection.
            Keep trying different axes till it is not violating ANY constraint.'''
            for constraint in unsafe_box:
                slacks = np.concatenate([cur_pos - constraint[0], constraint[1] - cur_pos])
                if np.min(slacks) >= 0.:
                    print('too close to the hook!')
                    num_tries = 0
                    while _violate(cur_pos) and num_tries < 3:
                        # reset pos before trying another dimension
                        cur_pos = self.normalize_ee_obs(np.concatenate([pos, [0.]]))[:3]
                        min_idx = np.argmin(slacks)
                        if min_idx // 3:
                            cur_pos[min_idx % 3] +=  (slacks[min_idx] + 1e-2)
                        else:
                            cur_pos[min_idx % 3] -= (slacks[min_idx] + 1e-2)
                        slacks[min_idx % 3] = slacks[min_idx % 3 + 3] = np.inf
                        num_tries += 1
                    return self.unnormalize_ee_obs(np.concatenate([cur_pos, [0.]]))[:3], gripper
        return pos, gripper

    def _update_robot(self, pos, angle, gripper):
        """input: the commanded position (clipped before).
        feasible position (based on forward kinematics) is tracked and used for updating,
        but the real position is used in observation."""
        feasible_pos, feasible_angle = self._robot.update_pose(pos, angle)
        self._robot.update_gripper(gripper)
        self._desired_pose = {'position': feasible_pos, 
                              'angle': feasible_angle,
                              'gripper': gripper}

    @property
    def _curr_pos(self):
        if self.use_desired_pose: return self._desired_pose['position'].copy()
        return self._robot.get_ee_pos()

    @property
    def _curr_angle(self):
        if self.use_desired_pose: return self._desired_pose['angle'].copy()
        return self._robot.get_ee_angle()

    def get_images(self):
        camera_feed = []
        if self._use_local_cameras:
            camera_feed.extend(self._camera_reader.read_cameras())
        else:
            camera_feed.extend(self._robot.read_cameras())
        return camera_feed

    def get_state(self):
        state_dict = {}
        gripper_state = self._robot.get_gripper_state()

        state_dict['control_key'] = 'desired_pose' if \
            self.use_desired_pose else 'current_pose'

        state_dict['desired_pose'] = np.concatenate(
            [self._desired_pose['position'],
            self._desired_pose['angle'],
            [self._desired_pose['gripper']]])

        state_dict['current_pose'] = np.concatenate(
            [self._robot.get_ee_pos(),
            self._robot.get_ee_angle(),
            [gripper_state]])

        state_dict['joint_positions'] = self._robot.get_joint_positions()
        state_dict['joint_velocities'] = self._robot.get_joint_velocities()
        # don't track gripper velocity
        state_dict['gripper_velocity'] = 0

        return state_dict

    def _randomize_reset_pos(self):
        '''takes random action along x-y plane, no change to z-axis / gripper'''
        random_xy = np.random.uniform(-0.5, 0.5, (2,))
        random_z = np.random.uniform(-0.2, 0.2, (1,))
        if self.DoF == 4:
            random_rot = np.random.uniform(-0.5, 0., (1,))
            act_delta = np.concatenate([random_xy, random_z, random_rot, np.zeros((1,))])
        else:
            act_delta = np.concatenate([random_xy, random_z, np.zeros((1,))])
        
        print(act_delta)
        for _ in range(10):
            self.step(act_delta)

    def render(self, mode=None):
        # TODO: update rendering to use height, width (for high quality evaluation rendering)
        if mode == 'video':
            image_obs = self.get_images()
            obs = np.concatenate([image_obs[0]['array'],
                                  image_obs[1]['array']], axis=0)
            return obs
        else:
            return self.get_observation()

    def is_robot_reset(self, epsilon=0.1):
        curr_joints = self._robot.get_joint_positions()
        joint_dist = np.linalg.norm(curr_joints - self._reset_joint_qpos)
        return joint_dist < epsilon

    @property
    def num_cameras(self):
        return len(self.get_images())
    