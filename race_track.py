import numpy as np
from gym import spaces
from gym import Env


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
        return None
        
    def reset(self):
        self._track = Track(self.miu, self.dot_miu)

        self._carState_main = CarState(startRanking = 1)
        self._car_main = Car(self._carState_main, 1)
        self.track.initializeCar(self._carState_main)

        return self._get_obs()

    def _get_obs(self):
        return self._carState_main.getObservation(self._track)

    def step(self, action, i = 0):
        return self._carState_main.step(ac[0], ac[1], self._track, i)

    def viewer_setup(self):
        print('no viewer')
        pass

    def render(self):
        print('current state:', self._state)

    def seed(self, seed):
        np.random.seed = seed