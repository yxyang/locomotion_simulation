"""Estimates base velocity for A1 robot from accelerometer readings."""
import numpy as np


class VelocityEstimator:
  """Estimates base velocity of A1 robot.

  Two sources of information are used:
  The integrated reading of accelerometer and the velocity estimation from
  contact legs. The readings are fused together using a Kalman Filter.
  """
  def __init__(self, time_step, pybullet_client):
    self.velocity = np.zeros(3)
    self.time_step = time_step
    self.pybullet_client = pybullet_client

  def reset(self):
    self.velocity = np.zeros(3)

  def update(self, robot_state):
    """Propagate current state estimate with new accelerometer reading."""
    sensor_acc = np.array(robot_state.imu.accelerometer)
    q = robot_state.imu.quaternion
    base_orientation = [q[1], q[2], q[3], q[0]]
    rot_mat = self.pybullet_client.getMatrixFromQuaternion(base_orientation)
    rot_mat = np.array(rot_mat).reshape((3, 3))
    calibrated_acc = rot_mat.dot(sensor_acc) + np.array([0., 0., -9.8])
    self.velocity += self.time_step * calibrated_acc
    # TODO: correct estimation using contact legs

  @property
  def estimated_velocity(self):
    return self.velocity
