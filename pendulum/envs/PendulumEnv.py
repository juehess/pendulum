import gym
from gym import spaces
import numpy as np
import math
class PendulumEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self, arg1, arg2, ...):
    super(PendulumEnv, self).__init__()

    self.max_motor_cmd = 200

    # Angle limit set to 2 * theta_threshold_radians so failing observation
    # is still within bounds.
    high = np.array(
      [
        np.pi,
        np.finfo(np.float32).max,
        np.pi,
        np.finfo(np.float32).max,
      ],
      dtype=np.float32,
    )

    # observation: cart position, cart velocity, pole angle, pole angular velocity
    self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    self.action_space = spaces.Box(
      low=-self.max_motor_cmd, high=self.max_motor_cmd, shape=(1,), dtype=np.int
    )



  def step(self, action):
    # Execute one time step within the environment
    ...
  def reset(self):
    # Reset the state of the environment to an initial state
    ...
  def render(self, mode='human', close=False):
    # Render the environment to the screen
    ...