import gym
from gym.envs.registration import registry, make, spec

def register(id, *args, **kvargs):
  if id in registry.env_specs:
    return
  else:
    return gym.envs.registration.register(id, *args, **kvargs)


register(
    id='A1GymEnv-v1',
    entry_point='locomotion.envs.gym_envs:A1GymEnv',
    max_episode_steps=2000,
    reward_threshold=2000.0,
)
