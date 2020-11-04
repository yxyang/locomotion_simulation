# Python Environments for Unitree A1 Robot

This is the simulated environment and real-robot interface for the A1 robot. The codebase can be installed directly as a PIP package, or cloned for further configurations.

The codebase also includes a whole-body controller that can walk the robot in both simulation and real world.

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
--on_rack=True
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

Note that the pybullet rendering is slightly different from Mujoco. To enable GUI rendering and visualize the training process, you can call:

```python
import gym
env = gym.make('locomotion:A1GymEnv-v1', render=True)
```

which will pop up the standard pybullet renderer.

And you can always call env.render(mode='rgb_array') to generate frames.

## Running on the real robot
Since the [SDK](https://github.com/unitreerobotics/unitree_legged_sdk) from Unitree is implemented in C++, we find the optimal way of robot interfacing to be via C++-python interface using pybind11.

### Step 1: Build and Test the robot interface

To start, build the python interface by running the following:
```bash
cd third_party/unitree_legged_sdk
mkdir build
cd build
cmake ..
make
```
Then copy the built `robot_interface.XXX.so` file to the main directory (where you can see this README.md file).

### Step 2: (Optional) Setup correct permissions for non-sudo user
Since the Unitree SDK requires memory locking and high process priority, root priority with `sudo` is usually required to execute commands. As an alternative, adding the following lines to `/etc/security/limits.confg` might allow you to run the SDK without `sudo`.

```
<username> soft memlock unlimited
<username> hard memlock unlimited
<username> soft nice eip
<username> hard nice eip
```

You may need to reboot the computer for the above changes to get into effect.

### Step 3: Test robot interface.

Test the python interfacing by running:
`python -m locomotion.examples.test_robot_interface`

If the previous steps were completed correctly, the script should finish without throwing any errors.

Note that this code does *not* do anything on the actual robot.

It's also recommended to try running:
`python -m locomotion.examples.a1_robot_exercise`

which executes open-loop sinusoidal position commands so that the robot can stand up and down.

## Running the Whole-body controller

To see the whole-body controller, run:
```bash
python -m locomotion.examples.whole_body_controller_example --use_gamepad=False --show_gui=True --use_real_robot=False --max_time_secs=10
```

There are 4 commandline flags:

`use_real_robot`: `True` for using the real robot, `False` for using the simulator.

`show_gui`: (simulation only) whether to visualize the simulated robot in GUI.

`use_gamepad`: whether to control the robot using a gamepad (e.g. Logitech F710), or let the robot follow a demo trajectory.

`max_time_secs`: the amount of time to execute the controller. For real robot testing, it's recommended to start with a small value of `max_time_secs` and gradually increase it.

## Credits

The codebase is derived from the Laikago simulation environment in the [motion_imitation](https://github.com/google-research/motion_imitation) project.

The underlying simulator used is [Pybullet](https://pybullet.org/wordpress/).
