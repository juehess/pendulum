import gymnasium as gym
from gym import spaces
import numpy as np
import math

from gym import spaces
from gym.utils import seeding

import pyglet

from pendulum.pendulum import Pendulum
class PendulumEnv(gym.Env):
  """Custom Environment that follows gym interface"""
  metadata = {'render.modes': ['human']}

  def __init__(self):
    super(PendulumEnv, self).__init__()

    self.max_motor_command = 200
    #initialize hardware and calibrate it
    self.pendulum = Pendulum()
    self.pendulum.reset()

    #self.action_space = spaces.Discrete(3)  # -1, 0, 1
    self.action_space = gym.spaces.Box(low=-200.0, high=200.0, shape=(1,), dtype=np.float32)
    self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4,))


  def reset(self):
    self.pendulum.reset()
    state = self.pendulum.getState()
    return np.array(state)

  def step(self, action):
    assert self.action_space.contains(action), "Invalid action"

    # Take action
    if action == 0:
      self.pendulum.setSpeed(0)
    elif action == 1:
      self.pendulum.setSpeed(70)
    elif action == 2:
      self.pendulum.setSpeed(-70)

    # Compute new state
    self.pendulum.computeState()
    state = self.pendulum.getState()
    observation = np.array(state)

    # Compute reward
    reward = computeReward(observation)

    # Check if episode is done
    done = abs(observation[2]) > 0.5

    return observation, reward, done, {}

  def render(self, mode='human'):
    screen_width = 600
    screen_height = 400

    world_width = self.x_threshold * 2
    scale = screen_width / world_width
    cartwidth = 50.0
    cartheight = 30.0

    if self.viewer is None:
      self.viewer = pyglet.window.Window(screen_width, screen_height)
      self.viewer.set_caption("Inverted Pendulum")

      self.track = pyglet.graphics.Batch()
      self.cart = pyglet.graphics.Batch()

      l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
      axleoffset = cartheight / 4.0
      cart = pyglet.graphics.vertex_list(4, ('v2f', [l, b, l, t, r, t, r, b]), ('c3B', (86, 109, 249) * 4))
      l, r, t, b = -2.5, 2.5, 2.5, -2.5
      axle = pyglet.graphics.vertex_list(4, ('v2f', [l, b, l, t, r, t, r, b]), ('c3B', (128, 128, 128) * 4))
      self.cart.add(4, pyglet.gl.GL_QUADS, cart, ('v2f', (0, 0) * 4), ('c3B', (255, 255, 255) * 4))
      self.cart.add(4, pyglet.gl.GL_QUADS, axle, ('v2f', (0, 0) * 4), ('c3B', (0, 0, 0) * 4))

      self.poletrans = pyglet.graphics.Batch()
      l, r, t, b = -self.polewidth / 2, self.polewidth / 2, self.polelen - self.polewidth / 2, -self.polewidth / 2
      self.pole = self.poletrans.add(4, pyglet.gl.GL_QUADS, None, ('v2f', [l, b, l, t, r, t, r, b]),
                                     ('c3B', (240, 197, 77) * 4))

      self.axle = self.poletrans.add(4, pyglet.gl.GL_QUADS, None, ('v2f',
                                                                   [-4 * axleoffset, 4 * axleoffset, -axleoffset,
                                                                    axleoffset, axleoffset, axleoffset,
                                                                    4 * axleoffset, -axleoffset]),
                                     ('c3B', (0, 0, 0) * 4))
      self.poletrans = self.poletrans.rotate(-90)

    if self.state is None:
      return None

    x = self.state
    cartx = x[0] * scale + screen_width / 2.0
    self.cart.vertices = [cartx + lx for lx in [-cartwidth / 2, -cartwidth / 2, cartwidth / 2, cartwidth / 2]]
    self.poletrans.position = (cartx, cartheight)
    self.poletrans.rotation = np.rad2deg(-x[2])

    self.viewer.clear()
    self.viewer.draw_rect(0, 0, screen_width, screen_height, color=(255, 255, 255))
    self.track.draw()

    self.viewer.add_geom(self.cart)
    self.viewer.add_geom(self.poletrans)

    return self.viewer.render()

 def close(self):
   pass

 def computeReward(self,observation):
   reward = -self.theta ** 2 - 0.1 * self.dtheta ** 2 - 0.001 * (u ** 2)
   return reward