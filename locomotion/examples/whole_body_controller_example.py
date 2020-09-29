"""Example of whole body controller on A1 robot."""
from absl import app
from absl import flags
import numpy as np
import os
import scipy.interpolate
import time

from locomotion.agents.mpc_controller import com_velocity_estimator
from locomotion.agents.mpc_controller import gait_generator as gait_generator_lib
from locomotion.agents.mpc_controller import locomotion_controller
from locomotion.agents.mpc_controller import openloop_gait_generator
from locomotion.agents.mpc_controller import raibert_swing_leg_controller
from locomotion.agents.mpc_controller import torque_stance_leg_controller

from locomotion.envs import env_builder
from locomotion.robots import a1
from locomotion.robots import robot_config

flags.DEFINE_string("logdir", None, "where to log trajectories.")
FLAGS = flags.FLAGS

_NUM_SIMULATION_ITERATION_STEPS = 300

_STANCE_DURATION_SECONDS = [
    0.3
] * 4  # For faster trotting (v > 1.5 ms reduce this to 0.13s).
_DUTY_FACTOR = [0.6] * 4
_INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]
_MAX_TIME_SECONDS = 5
_MOTOR_KD = [1.0, 2.0, 2.0] * 4

LAIKAGO_STANDING = (
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
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
      initial_leg_phase=_INIT_PHASE_FULL_CYCLE)
  state_estimator = com_velocity_estimator.COMVelocityEstimator(robot)
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
      desired_body_height=robot.MPC_BODY_HEIGHT,
      body_mass=robot.MPC_BODY_MASS,
      body_inertia=robot.MPC_BODY_INERTIA)

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
  env = env_builder.build_regular_env(
      a1.A1,
      motor_control_mode=robot_config.MotorControlMode.HYBRID,
      enable_rendering=True,
      on_rack=False,
      wrap_trajectory_generator=False)
  robot = env.robot
  p = env.pybullet_client

  controller = _setup_controller(robot)
  controller.reset()

  current_time = robot.GetTimeSinceReset()
  actions = []
  while current_time < max_time:
    # Updates the controller behavior parameters.
    lin_speed, ang_speed = _generate_example_linear_angular_speed(current_time)
    _update_controller_params(controller, lin_speed, ang_speed)

    # Needed before every call to get_action().
    controller.update()
    hybrid_action = controller.get_action()
    actions.append(hybrid_action)
    robot.Step(hybrid_action)
    time.sleep(0.007)

    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)

    current_time = robot.GetTimeSinceReset()

  if FLAGS.logdir:
    np.savez(os.path.join(FLAGS.logdir, 'action.npz'), action=actions)


def main(argv):
  del argv
  _run_example()


if __name__ == "__main__":
  app.run(main)
