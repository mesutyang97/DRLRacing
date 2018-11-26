import numpy as np
from gym import spaces
from gym import Env
from track import *


class RaceTrackEnv(Env):
    """
    """

    def __init__(self, num_cars=1, miu = 0.8, dot_miu = 0.3, env_window_w = 500, obs_window_w = 10):
        self.num_cars = num_cars
        self.miu = miu
        self.dot_miu = dot_miu
        self.env_window_w = env_window_w
        self.obs_window_w = obs_window_w

        self.reset()
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_window_w * obs_window_w + 4,))
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,))

    def reset_task(self, is_evaluation=False):
        self.enablePrint = is_evaluation
        # self.enablePrint = True
        return None

    def reset(self):
        self._track = Track(self.miu, self.dot_miu)

        self._carState_main = CarState(startRanking = 1)
        self._car_main = Car(self._carState_main, 1)
        self._track.initializeCar(self._carState_main)
        self.count = 0

        return self._get_obs()

    def _get_obs(self):
        return self._carState_main.getObservation(self._track)

    def step(self, action, i = 0):
        self.count += 1
        print ("Steped", self.count)
        if self.count % 5 == 0:
            self._track.rebuildTrack()
        action = action.flatten()
        # Enforce 1.0 Throttle for now
        return self._carState_main.step(action[0], action[1], self._track, i, self.enablePrint)

    def viewer_setup(self):
        print('no viewer')
        pass

    def render(self):
        print('current state:', self._state)

    def seed(self, seed):
        np.random.seed = seed