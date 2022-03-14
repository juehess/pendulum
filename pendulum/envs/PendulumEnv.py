import gym
from gym import spaces
import numpy as np

class PendulumEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self, arg1, arg2, ...):
    super(PendulumEnv, self).__init__()

    N_DISCRETE_ACTIONS = 510
    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions:
    self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)

    # Example for using image as input:
    self.observation_space = spaces.Box(
        low=np.array([0, 0]), high=np.array([3, 1]), dtype=np.float16)

  def step(self, action):
    # Execute one time step within the environment
    ...
  def reset(self):
    # Reset the state of the environment to an initial state
    ...
  def render(self, mode='human', close=False):
    # Render the environment to the screen
    ...