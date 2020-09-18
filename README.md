# Environment for A1 and Laikago Robot Simulation

This is the simulated environment for the A1 and Laikago quadruped robot. The codebase can be installed directly as a PIP package, or cloned for further configurations.

## Getting started
To start, just clone the codebase, and install the dependencies using
```bash
pip install -r requirements.txt
```

Then, you can explore the environments by running:
```bash
python -m locomotion.examples.test_env_gui \
--robot_type=A1 \
--motor_control_mode=Position \
--on_rack=False
```

The three commandline flags are:

`robot_type`: choose between `A1` and `Laikago` for different robot.

`motor_control_mode`: choose between `Position` ,`Torque` for different motor control modes.

`on_rack`: whether to fix the robot's base on a rack. Setting `on_rack=True` is handy for debugging visualizing open-loop gaits.

## The gym interface
Additionally, the codebase can be directly installed as a pip package. Just run:
```bash
pip install git+https://github.com/yxyang/locomotion_simulation@master#egg=locomotion_simulation
```

Then, you can directly invoke the default gym environment in Python:
```python
import gym
env = gym.make('locomotion:A1GymEnv-v1')
```

## Credits

The codebase is derived from the Laikago simulation environment in the [motion_imitation](https://github.com/google-research/motion_imitation) project.

The underlying simulator used is [Pybullet](https://pybullet.org/wordpress/).
