"""Apply the same action to the simulated and real A1 robot.


As a basic debug tool, this script allows you to execute the same action
(which you choose from the pybullet GUI) on the simulation and real robot
simultaneouly. Make sure to put the real robbot on rack before testing.
"""

from absl import app
from absl import flags
from absl import logging
import numpy as np
import pybullet as p  # pytype: disable=import-error
import time
from tqdm import tqdm

from locomotion.envs import env_builder
from locomotion.robots import a1
from locomotion.robots import a1_robot
from locomotion.robots import robot_config

FREQ = 0.5


def main(_):
  print("WARNING: this code executes low-level controller on the robot.")
  print("Make sure the robot is hang on rack before proceeding.")
  input("Press enter to continue...")

  # Construct sim env and real robot
  robot = a1_robot.A1(pybullet_client=None)
  while not robot.GetMotorAngles():
    print("Robot sensors not ready, sleep for 1s...")
    time.sleep(1)

  # Move the motors slowly to initial position
  current_motor_angle = np.array(robot.GetMotorAngles())
  desired_motor_angle = np.array([0., 0.9, -1.8] * 4)
  print(current_motor_angle)

  for t in tqdm(range(200)):
    blend_ratio = np.minimum(t / 100., 1)
    action = blend_ratio * current_motor_angle + (
        1 - blend_ratio) * desired_motor_angle
    robot.ApplyAction(action, robot_config.MotorControlMode.POSITION)
    time.sleep(0.01)
  print(robot.GetMotorAngles())

  # Move the legs in a sinusoidal curve
  for t in range(10000):
    angle_hip = 0.9 + 0.2 * np.sin(2 * np.pi * FREQ * 0.01 * t)
    angle_calf = -2 * angle_hip
    action = np.array([0., angle_hip, angle_calf] * 4)
    robot.ApplyAction(action, robot_config.MotorControlMode.POSITION)
    time.sleep(0.01)

  robot.Terminate()


if __name__ == '__main__':
  app.run(main)
