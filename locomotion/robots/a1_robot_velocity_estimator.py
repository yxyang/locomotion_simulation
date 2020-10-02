"""Estimates base velocity for A1 robot from accelerometer readings."""
import numpy as np
from filterpy.kalman import KalmanFilter


class VelocityEstimator:
  """Estimates base velocity of A1 robot.

  Two sources of information are used:
  The integrated reading of accelerometer and the velocity estimation from
  contact legs. The readings are fused together using a Kalman Filter.
  """
  def __init__(self,
               robot,
               accelerometer_variance=0.1,
               sensor_variance=0.1,
               initial_variance=0.1):
    """Initiates the velocity estimator.

    See filterpy documentation in the link below for more details.
    https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html

    Args:
      robot: the robot class for velocity estimation.
      accelerometer_variance: noise estimation for accelerometer reading.
      sensor_variance: noise estimation for motor velocity reading.
      initial_covariance: covariance estimation of initial state.
    """
    self.robot = robot

    self.filter = KalmanFilter(dim_x=3, dim_z=3, dim_u=3)
    self.filter.x = np.zeros(3)
    self._initial_variance = initial_variance
    self.filter.P = np.eye(3) * self._initial_variance  # State covariance
    self.filter.Q = np.eye(3) * accelerometer_variance
    self.filter.R = np.eye(3) * sensor_variance

    self.filter.H = np.eye(3)  # measurement function (y=H*x)
    self.filter.F = np.eye(3)  # state transition matrix
    self.filter.B = np.eye(
        3) * self.robot.time_step  # Control transition matrix

  def reset(self):
    self.filter.x = np.zeros(3)
    self.filter.P = np.eye(3) * self._initial_variance

  def update(self, robot_state):
    """Propagate current state estimate with new accelerometer reading."""
    sensor_acc = np.array(robot_state.imu.accelerometer)
    base_orientation = self.robot.GetBaseOrientation()
    rot_mat = self.robot.pybullet_client.getMatrixFromQuaternion(
        base_orientation)
    rot_mat = np.array(rot_mat).reshape((3, 3))
    calibrated_acc = rot_mat.dot(sensor_acc) + np.array([0., 0., -9.8])
    self.filter.predict(u=calibrated_acc)

    # TODO: correct estimation using contact legs
    observed_velocities = []
    foot_contact = self.robot.GetFootContacts()
    for leg_id in range(4):
      if foot_contact[leg_id]:
        jacobian = self.robot.ComputeJacobian(leg_id)
        # Only pick the jacobian related to joint motors
        com_dof = 6
        jacobian = jacobian[:, com_dof + leg_id * 3:com_dof + (leg_id + 1) * 3]
        joint_velocities = self.robot.motor_velocities[leg_id *
                                                       3:(leg_id + 1) * 3]
        leg_velocity_in_base_frame = jacobian.dot(joint_velocities)
        base_velocity_in_base_frame = -leg_velocity_in_base_frame[:3]
        observed_velocities.append(rot_mat.dot(base_velocity_in_base_frame))

    if observed_velocities:
      observed_velocities = np.mean(observed_velocities, axis=0)
      self.filter.update(observed_velocities)

  @property
  def estimated_velocity(self):
    return self.filter.x.copy()
