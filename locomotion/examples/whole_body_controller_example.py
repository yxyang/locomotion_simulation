"""Example of whole body controller on A1 robot."""
from absl import app
from absl import flags
from absl import logging
from datetime import datetime
import numpy as np
import os
import scipy.interpolate
import time

import pybullet_data
from pybullet_utils import bullet_client
import pybullet  # pytype:disable=import-error

from locomotion.agents.whole_body_controller import com_velocity_estimator
from locomotion.agents.whole_body_controller import gait_generator as gait_generator_lib
from locomotion.agents.whole_body_controller import locomotion_controller
from locomotion.agents.whole_body_controller import openloop_gait_generator
from locomotion.agents.whole_body_controller import raibert_swing_leg_controller
from locomotion.agents.whole_body_controller import torque_stance_leg_controller

# from locomotion.envs import env_builder
from locomotion.robots import a1
from locomotion.robots import robot_config

flags.DEFINE_string("logdir", None, "where to log trajectories.")
FLAGS = flags.FLAGS

_NUM_SIMULATION_ITERATION_STEPS = 300
_MAX_TIME_SECONDS = 30.

_STANCE_DURATION_SECONDS = [
    0.3
] * 4  # For faster trotting (v > 1.5 ms reduce this to 0.13s).

# Standing
# _DUTY_FACTOR = [1.] * 4
# _INIT_PHASE_FULL_CYCLE = [0., 0., 0., 0.]

# _INIT_LEG_STATE = (
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
# )

# Tripod
# _DUTY_FACTOR = [.8] * 4
# _INIT_PHASE_FULL_CYCLE = [0., 0.25, 0.5, 0.]

# _INIT_LEG_STATE = (
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.SWING,
# )

# Trotting
_DUTY_FACTOR = [0.6] * 4
_INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]

_INIT_LEG_STATE = (
    gait_generator_lib.LegState.SWING,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.SWING,
)


def _generate_example_linear_angular_speed(t):
  """Creates an example speed profile based on time for demo purpose."""
  vx = 0.6
  vy = 0.2
  wz = 0.8

  time_points = (0, 5, 10, 15, 20, 25, 30)
  speed_points = ((0, 0, 0, 0), (0, 0, 0, wz), (vx, 0, 0, 0), (0, 0, 0, -wz),
                  (0, -vy, 0, 0), (0, 0, 0, 0), (0, 0, 0, wz))

  speed = scipy.interpolate.interp1d(time_points,
                                     speed_points,
                                     kind="previous",
                                     fill_value="extrapolate",
                                     axis=0)(t)

  return speed[0:3], speed[3]


def _setup_controller(robot):
  """Demonstrates how to create a locomotion controller."""
  desired_speed = (0, 0)
  desired_twisting_speed = 0

  gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
      robot,
      stance_duration=_STANCE_DURATION_SECONDS,
      duty_factor=_DUTY_FACTOR,
      initial_leg_phase=_INIT_PHASE_FULL_CYCLE,
      initial_leg_state=_INIT_LEG_STATE)
  state_estimator = com_velocity_estimator.COMVelocityEstimator(robot,
                                                                window_size=20)
  sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_height=robot.MPC_BODY_HEIGHT,
      foot_clearance=0.01)

  st_controller = torque_stance_leg_controller.TorqueStanceLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_body_height=robot.MPC_BODY_HEIGHT)

  controller = locomotion_controller.LocomotionController(
      robot=robot,
      gait_generator=gait_generator,
      state_estimator=state_estimator,
      swing_leg_controller=sw_controller,
      stance_leg_controller=st_controller,
      clock=robot.GetTimeSinceReset)
  return controller


def _update_controller_params(controller, lin_speed, ang_speed):
  controller.swing_leg_controller.desired_speed = lin_speed
  controller.swing_leg_controller.desired_twisting_speed = ang_speed
  controller.stance_leg_controller.desired_speed = lin_speed
  controller.stance_leg_controller.desired_twisting_speed = ang_speed


def _run_example(max_time=_MAX_TIME_SECONDS):
  """Runs the locomotion controller example."""
  # env = env_builder.build_regular_env(
  #     a1.A1,
  #     motor_control_mode=robot_config.MotorControlMode.HYBRID,
  #     enable_rendering=True,
  #     on_rack=False,
  #     wrap_trajectory_generator=False)
  # robot = env.robot
  # p = env.pybullet_client
  p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
  p.setPhysicsEngineParameter(numSolverIterations=30)
  p.setTimeStep(0.001)
  p.setGravity(0, 0, -9.8)
  p.setPhysicsEngineParameter(enableConeFriction=0)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.loadURDF("plane.urdf")

  robot = a1.A1(p,
                motor_control_mode=robot_config.MotorControlMode.HYBRID,
                enable_action_interpolation=False,
                reset_time=2,
                action_repeat=5)

  controller = _setup_controller(robot)
  controller.reset()

  if FLAGS.logdir:
    logdir = os.path.join(FLAGS.logdir,
                          datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
    os.makedirs(logdir)
    video_path = os.path.join(logdir, 'video.mp4')
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_path)

  current_time = robot.GetTimeSinceReset()
  com_vels, imu_rates, actions = [], [], []
  while current_time < max_time:
    start_time_robot = current_time
    start_time_wall = time.time()
    # Updates the controller behavior parameters.
    lin_speed, ang_speed = _generate_example_linear_angular_speed(current_time)
    _update_controller_params(controller, lin_speed, ang_speed)
    controller.update()
    time_before = time.time()
    hybrid_action, _ = controller.get_action()
    print(time.time() - time_before)
    com_vels.append(np.array(robot.GetBaseVelocity()).copy())
    imu_rates.append(np.array(robot.GetBaseRollPitchYawRate()).copy())
    actions.append(hybrid_action)
    robot.Step(hybrid_action)

    current_time = robot.GetTimeSinceReset()
    expected_duration = current_time - start_time_robot
    actual_duration = time.time() - start_time_wall
    if actual_duration < expected_duration:
      time.sleep(expected_duration - actual_duration)

  if FLAGS.logdir:
    p.stopStateLogging(log_id)
    np.savez(os.path.join(logdir, 'action.npz'),
             action=actions,
             com_vels=com_vels,
             imu_rates=imu_rates)
    logging.info("logged to: {}".format(logdir))


def main(argv):
  del argv
  _run_example()


if __name__ == "__main__":
  app.run(main)
