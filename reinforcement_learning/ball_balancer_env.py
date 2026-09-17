# Gymnasium environment of the ball balancing robot, simulated in MuJoCo.
#
# The policy only has to stabilize the ball: its action is the slope u of the plate,
# the same quantity returned by control.pid and control.lqr, and everything after it
# is the chain that runs on the robot:
#   u -> kinematics.solve -> clip to [Q_MIN, Q_MAX] -> servo -> plate pose
# The arms are not simulated: the pose that the servo angles give to the plate is found
# with the forward kinematics, and the plate is driven kinematically through its joints.
# The observation is what the camera loop measures: the pixel error of the ball,
# delayed and noisy, together with the last actions.

import os
import sys
from collections import deque

import mujoco
import numpy as np
from gymnasium import spaces
from gymnasium.envs.mujoco import MujocoEnv

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import kinematics  # noqa: E402

# ACTION
U_MAX = 0.2                              # slope at action 1, the largest reachable in every direction with q in [Q_MIN, Q_MAX]

# ACTUATION, as in scripts/hardware.py
Q_MIN, Q_MAX = 45.0, 75.0

# CAMERA, as in scripts/vision.py
W, H = 320, 240                          # size of the camera frame
R_BALL = 0.02                            # ball radius in meters
PX_PER_M = 50.0 / R_BALL                 # the ball radius is 50 px in the frame

# OBSERVATION
N_ERR = 3                                # last errors in the observation, the velocity is not measured
N_ACT = 2                                # last actions in the observation, they carry what the delay hides
ERR_SCALE = W / 2                        # pixels -> observation

# REWARD
W_ACT = 0.05                             # weight of the action magnitude
W_DACT = 0.1                             # weight of the action change, the servos do not like chattering

# FORWARD KINEMATICS
FK_ITERS = 10
FK_TOL = 1e-3                            # deg
POSE_NOM = np.array([0.0, 0.0, kinematics.H_NOM])    # (u_x, u_y, h), h in mm


def _jacobian(pose, eps=(1e-4, 1e-4, 1e-2)):
    # d q / d (u_x, u_y, h) by central differences, q in degrees
    cols = []
    for i in range(3):
        d = np.zeros(3)
        d[i] = eps[i]
        qp = kinematics.solve((pose + d)[:2], (pose + d)[2])
        qm = kinematics.solve((pose - d)[:2], (pose - d)[2])
        cols.append((qp - qm) / (2 * eps[i]))
    return np.column_stack(cols)


G_NOM = np.linalg.inv(_jacobian(POSE_NOM))  # d pose / d q at the nominal pose


def forward_kinematics(q, pose0=POSE_NOM):
    # motor angles in degrees -> pose (u_x, u_y, h) of the plate.
    # quasi Newton on the inverse kinematics, with the Jacobian of the nominal pose:
    # a warm start from the previous pose converges in two or three iterations
    pose = np.array(pose0, dtype=float)
    for _ in range(FK_ITERS):
        q_pose = kinematics.solve(pose[:2], pose[2])
        if q_pose is None:
            break
        dq = q - q_pose
        if np.max(np.abs(dq)) < FK_TOL:
            break
        pose = pose + G_NOM @ dq
    return pose


def plate_angles(pose):
    # plate normal n = [u_x, u_y, 1] -> joint angles, with R = Ry(ry) Rx(rx)
    n = np.array([pose[0], pose[1], 1.0])
    n = n / np.linalg.norm(n)
    return -np.arcsin(n[1]), np.arctan2(n[0], n[2])


# PHYSICAL PARAMETERS: nominal values and ranges of the domain randomization
NOMINAL = dict(
    substeps=23,                         # physics steps per control step, 46 ms: the loop runs at about 22 Hz
    latency=0.04,                        # s, from the frame to the command of the servos
    noise_px=1.0,                        # std of the detected ball position
    servo_tau=0.025,                     # s, time constant of the servos
    servo_speed=500.0,                   # deg/s, the SG90 does 60 deg in 0.1 s without load
    deadband=0.5,                        # deg
    q_offset=np.zeros(3),                # deg, what OFFSETS in hardware.py do not correct of the horns
    cam_yaw=0.0,                         # deg, rotation of the camera about the vertical
    cam_offset=np.zeros(2),              # px, frame centre with respect to the plate centre
    px_per_m=PX_PER_M,
    ball_mass=0.0027,                    # kg
    slide_friction=0.4,
    roll_friction=2e-5,                  # m
)
RANDOM = dict(
    substeps=(20, 27),
    latency=(0.02, 0.06),
    noise_px=(0.5, 2.0),
    servo_tau=(0.015, 0.04),
    servo_speed=(300.0, 600.0),
    deadband=(0.0, 1.0),
    q_offset=(-2.0, 2.0),
    cam_yaw=(-5.0, 5.0),
    cam_offset=(-5.0, 5.0),
    px_per_m=(0.9 * PX_PER_M, 1.1 * PX_PER_M),
    ball_mass=(0.0025, 0.0029),
    slide_friction=(0.2, 0.8),
    roll_friction=(0.0, 5e-5),
)

DEFAULT_CAMERA_CONFIG = dict(distance=0.45, azimuth=135.0, elevation=-25.0, lookat=np.array([0.0, 0.0, 0.04]))


class BallBalancerEnv(MujocoEnv):
    metadata = {"render_modes": ["human", "rgb_array", "depth_array"], "render_fps": 22}

    def __init__(self, randomize=True, max_episode_steps=440, render_mode=None,
                 width=480, height=360, camera_id=None, camera_name=None):
        self.randomize = randomize
        self.max_episode_steps = max_episode_steps      # 440 steps, about 20 s
        observation_space = spaces.Box(-np.inf, np.inf, shape=(2 * N_ERR + 2 * N_ACT,), dtype=np.float32)
        super().__init__(
            os.path.join(HERE, "ball_balancer.xml"),
            frame_skip=NOMINAL["substeps"],
            observation_space=observation_space,
            render_mode=render_mode,
            width=width,
            height=height,
            camera_id=camera_id,
            camera_name=camera_name,
            default_camera_config=None if camera_id is not None or camera_name is not None else DEFAULT_CAMERA_CONFIG,
        )
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        self._ball_body = self.model.body("ball").id
        self._ball_geom = self.model.geom("ball").id
        self._plate_geom = self.model.geom("plate").id
        self._ball_qpos = self.model.jnt_qposadr[self.model.joint("ball").id]
        self._ball_qvel = self.model.jnt_dofadr[self.model.joint("ball").id]
        self._px_buffer = deque(maxlen=int(RANDOM["latency"][1] / self.model.opt.timestep) + 2)

    # PARAMETERS
    def _sample_params(self):
        if not self.randomize:
            return {k: np.copy(v) for k, v in NOMINAL.items()}
        p = {"substeps": RANDOM["substeps"]}     # drawn again at every step, the loop period jitters
        for k, (lo, hi) in RANDOM.items():
            if k != "substeps":
                p[k] = self.np_random.uniform(lo, hi, size=np.shape(NOMINAL[k]) or None)
        return p

    def _apply_params(self):
        p = self.params
        self.model.body_mass[self._ball_body] = p["ball_mass"]
        self.model.body_inertia[self._ball_body] = 2 / 3 * p["ball_mass"] * R_BALL**2    # hollow shell
        for g in (self._ball_geom, self._plate_geom):
            self.model.geom_friction[g] = [p["slide_friction"], 0.001, p["roll_friction"]]
        yaw = np.radians(p["cam_yaw"])
        self._cam_rot = p["px_per_m"] * np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        self._alpha = 1 - np.exp(-self.model.opt.timestep / p["servo_tau"])
        self._delay = int(round(p["latency"] / self.model.opt.timestep))

    # ACTUATION
    def _command(self, action):
        # the chain of ball_balancer.py: slope -> inverse kinematics -> clip
        u = action * U_MAX
        q = kinematics.solve(u, kinematics.H_NOM)
        if q is None:       # unreachable: as on the robot, the servos keep the previous command
            return
        q = np.clip(q, Q_MIN, Q_MAX)
        moved = np.abs(q - self._q_cmd) >= self.params["deadband"]
        self._q_cmd = np.where(moved, q, self._q_cmd)
        self._q_target = self._q_cmd + self.params["q_offset"]
        self._pose_target = forward_kinematics(self._q_target, self._pose_target)

    def _servo_step(self):
        # first order lag with a rate limit, per servo
        dt = self.model.opt.timestep
        max_step = self.params["servo_speed"] * dt
        self._q = self._q + np.clip((self._q_target - self._q) * self._alpha, -max_step, max_step)
        # close to the target the pose is linear in q, so the exact forward kinematics
        # is solved once per control step and not at every physics step
        self._set_plate(self._pose_target + G_NOM @ (self._q - self._q_target))

    def _set_plate(self, pose, velocity=True):
        rx, ry = plate_angles(pose)
        qpos = np.array([pose[2] / 1000.0, ry, rx])
        if velocity:
            self.data.qvel[:3] = (qpos - self.data.qpos[:3]) / self.model.opt.timestep
        else:
            self.data.qvel[:3] = 0.0
        self.data.qpos[:3] = qpos
        self._pose = pose

    # SENSING
    def _ball_px(self):
        # position of the ball in the frame, from the centre, in pixels
        xy = self.data.qpos[self._ball_qpos:self._ball_qpos + 2]
        return self._cam_rot @ xy + self.params["cam_offset"]

    def _in_view(self, px):
        return abs(px[0]) < W / 2 and abs(px[1]) < H / 2

    def _measure(self):
        # error = frame centre - ball, from the frame taken `latency` ago
        k = min(self._delay, len(self._px_buffer) - 1)
        px = self._px_buffer[-1 - k]
        return -px + self.np_random.normal(0.0, self.params["noise_px"], size=2)

    def _get_obs(self):
        return np.concatenate([self._err_hist.ravel() / ERR_SCALE, self._act_hist.ravel()]).astype(np.float32)

    # GYMNASIUM
    def reset_model(self):
        self.params = self._sample_params()
        self._apply_params()

        # servos at the command of the level plate
        self._q_cmd = np.clip(kinematics.solve([0.0, 0.0], kinematics.H_NOM), Q_MIN, Q_MAX)
        self._q_target = self._q_cmd + self.params["q_offset"]
        self._q = self._q_target.copy()
        self._pose_target = forward_kinematics(self._q_target)
        self._set_plate(self._pose_target, velocity=False)

        # ball at rest on the plate somewhere in view, possibly already rolling
        r = self.np_random.uniform(0.0, 0.03)
        phi = self.np_random.uniform(0.0, 2 * np.pi)
        x, y = r * np.cos(phi), r * np.sin(phi)
        n = np.array([self._pose[0], self._pose[1], 1.0])
        n = n / np.linalg.norm(n)
        z = self._pose[2] / 1000.0 - (n[0] * x + n[1] * y) / n[2]
        v = self.np_random.uniform(-0.05, 0.05, size=2)
        i, j = self._ball_qpos, self._ball_qvel
        self.data.qpos[i:i + 3] = np.array([x, y, z]) + (R_BALL + 1e-4) * n
        self.data.qpos[i + 3:i + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[j:j + 6] = [v[0], v[1], 0.0, -v[1] / R_BALL, v[0] / R_BALL, 0.0]    # rolling
        mujoco.mj_forward(self.model, self.data)

        self._px_buffer.clear()
        self._px_buffer.append(self._ball_px())
        err = self._measure()
        self._err_hist = np.tile(err, (N_ERR, 1))       # as control.reset on reacquisition
        self._act_hist = np.zeros((N_ACT, 2))
        self._steps = 0
        self._dt = NOMINAL["substeps"] * self.model.opt.timestep
        return self._get_obs()

    def _get_reset_info(self):
        return {"err": self._err_hist[0].copy(), "dt": self._dt, "params": self.params}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        self._command(action)

        if self.randomize:
            substeps = int(self.np_random.integers(RANDOM["substeps"][0], RANDOM["substeps"][1] + 1))
        else:
            substeps = NOMINAL["substeps"]
        for _ in range(substeps):
            self._servo_step()
            mujoco.mj_step(self.model, self.data)
            self._px_buffer.append(self._ball_px())
        self._dt = substeps * self.model.opt.timestep
        self._steps += 1

        px = self._px_buffer[-1]
        lost = not (np.all(np.isfinite(self.data.qpos)) and self._in_view(px))

        err = self._measure()
        self._err_hist = np.roll(self._err_hist, 1, axis=0)
        self._err_hist[0] = err
        a_prev = self._act_hist[0].copy()
        self._act_hist = np.roll(self._act_hist, 1, axis=0)
        self._act_hist[0] = action

        dist = np.linalg.norm(px) / (H / 2)
        reward = 1.0 - dist - W_ACT * np.sum(action**2) - W_DACT * np.sum((action - a_prev)**2)

        terminated = bool(lost)
        truncated = self._steps >= self.max_episode_steps
        info = {"err": err, "ball_px": px.copy(), "dt": self._dt, "q": self._q.copy(), "pose": self._pose.copy()}

        if self.render_mode == "human":
            self.render()
        return self._get_obs(), float(reward), terminated, truncated, info
