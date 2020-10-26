"""Set up the zeroth-order QP problem for stance leg control.

For details, please refer to section XX of this paper:
https://arxiv.org/abs/2009.10019
"""

import numpy as np
import scipy
import osqp
np.set_printoptions(precision=3, suppress=True)

ACC_WEIGHT = np.array([1., 1., 1., 10., 10, 1.])


def make_skew(x):
  return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


def compute_mass_matrix(robot):
  robot_mass = robot.MPC_BODY_MASS
  robot_inertia = np.array(robot.MPC_BODY_INERTIA).reshape((3, 3))
  foot_positions = robot.GetFootPositionsInBaseFrame()
  yaw = 0. # Set yaw to 0 for now as all commands are local.
  rot_z = np.array([[np.cos(yaw), np.sin(yaw), 0],
                    [-np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

  inv_mass = np.eye(3) / robot_mass
  inv_inertia = np.linalg.inv(robot_inertia)
  mass_mat = np.zeros((6, 12))

  for leg_id in range(4):
    mass_mat[:3, leg_id * 3:leg_id * 3 + 3] = inv_mass#.copy()
    mass_mat[3:6, leg_id * 3:leg_id * 3 + 3] = rot_z.T.dot(inv_inertia).dot(
        make_skew(foot_positions[leg_id]))
  return mass_mat


def compute_constraint_matrix(robot,
                              contacts,
                              friction_coef=0.8,
                              f_min_ratio=0.1,
                              f_max_ratio=10):
  f_min = f_min_ratio * robot.MPC_BODY_MASS * 9.8
  f_max = f_max_ratio * robot.MPC_BODY_MASS * 9.8
  A = np.zeros((20, 12))
  lb, ub = np.zeros(20), np.zeros(20)
  for leg_id in range(4):
    A[leg_id, leg_id * 3 + 2] = 1
    if contacts[leg_id]:
      lb[leg_id], ub[leg_id] = f_min, f_max
    else:
      lb[leg_id], ub[leg_id] = 0, 0

  # Friction constraints
  for leg_id in range(4):
    row_id = 4 + leg_id * 4
    col_id = leg_id * 3
    lb[row_id:row_id + 4] = np.array([0., 0, 0, 0])
    ub[row_id:row_id + 4] = np.array([np.inf, np.inf, np.inf, np.inf])
    A[row_id, col_id:col_id + 3] = np.array([1, 0, friction_coef])
    A[row_id + 1, col_id:col_id + 3] = np.array([-1, 0, friction_coef])
    A[row_id + 2, col_id:col_id + 3] = np.array([0, 1, friction_coef])
    A[row_id + 3, col_id:col_id + 3] = np.array([0, -1, friction_coef])
  return scipy.sparse.csc_matrix(A), lb, ub


def compute_objective_matrix(robot, desired_acc, acc_weight, reg_weight):
  M = compute_mass_matrix(robot)
  g = np.array([0., 0., 9.8, 0., 0., 0.])
  Q = np.diag(acc_weight)
  R = np.ones(12) * reg_weight

  quad_term = M.T.dot(Q).dot(M) + R
  linear_term = -1 * (g + desired_acc).T.dot(Q).dot(M)
  return scipy.sparse.csc_matrix(quad_term), linear_term


def compute_contact_force(robot,
                          desired_acc,
                          contacts,
                          acc_weight=ACC_WEIGHT,
                          reg_weight=1e-4,
                          friction_coef=0.45,
                          f_min_ratio=0.1,
                          f_max_ratio=10.):
  P, q = compute_objective_matrix(robot, desired_acc, acc_weight, reg_weight)
  A, l, u = compute_constraint_matrix(robot, contacts, friction_coef,
                                      f_min_ratio, f_max_ratio)
  result = osqp.solve(P=P, q=q, A=A, l=l, u=u, verbose=False)
  return -result.x.reshape((4, 3))
