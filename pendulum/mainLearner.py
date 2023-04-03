import gym
import numpy as np

env = gym.make('InvertedPendulum-v0')
obs = env.reset()

total_reward = 0.0
for t in range(1000):
    action = np.random.randint(env.action_space.n)
    obs, reward, done, _ = env.step(action)
    total_reward += reward
    if done:
        break

env.close()

print("Total reward: {}".format(total_reward))