from enum import Enum
import math
from scipy.interpolate import interp1d
import numpy as np


class GazeSmoothing():
    def __init__(self, window_size, logger, weights=None):
        self.logger = logger
        self.signal_picker = [False] * int(window_size)
        self.moving_window = np.zeros((window_size, 2))

    def compute_smoothed_signal(self, validflag, gazex=None, gazey=None):
        self.signal_picker.pop(0)
        self.signal_picker.append(validflag)
        self.moving_window = np.delete(self.moving_window, 0, 0)
        self.moving_window = np.concatenate((self.moving_window, np.array([[gazex, gazey]])))
        
        if True in self.signal_picker:
            valid_signals = self.moving_window[self.signal_picker, :]
            # self.logger.info("valid signals shape: {}".format(valid_signals.shape))
            smoothed_signal = (np.mean(valid_signals, axis=0)).flatten()
            return smoothed_signal
        else:
            return np.array([None, None])
