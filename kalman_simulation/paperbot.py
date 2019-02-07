import numpy as np
import matplotlib.pyplot as plt

class PaperBot:
    def __init__(self, x0, y0, th0):
        self.x = x0
        self.y = y0
        self.th = th0
        self.history = []
        self._add_history()

        self.t = 0.5
        self.d = 49
        self.A = 84


    def _add_history(self):
        self.history.append([self.x, self.y, self.th])


    def dynamics(self, wl, wr):

        dx = self.t / 2 * np.pi * np.cos(self.th) * (wl + wr) * self.d