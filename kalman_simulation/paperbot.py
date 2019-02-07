import numpy as np
import matplotlib.pyplot as plt
import  matplotlib.cm as cm

class PaperBot:
    def __init__(self, x0, y0, th0):
        self.x = x0
        self.y = y0
        self.th = th0
        self.history = []

        self.t = 0
        
        self.dt = 0.1
        self.d = 49
        self.A = 84


    def _add_history(self):
        self.history.append([self.x, self.y, self.th, self.t])
        self.t += 1


    def dynamics(self, wl, wr):
        dconst = self.dt / 2 * np.pi * self.d
        dx =  dconst * np.cos(self.th) * (wl + wr)
        dy = dconst * np.sin(self.th) * (wl + wr)
        dth = dconst / self.A * (wr - wl)
        return dx, dy, dth
    
    
    def move(self, ul, ur):
        print('t: {} -- X: {}, Y: {}, Th: {}'.format(self.t, self.x, self.y, self.th))
        print('\tul: {}, ur: {}'.format(ul, ur))
        ul -= 90
        ur -= 90
        
        wl = np.sign(ul) * np.power(np.abs(ul), 0.2)
        wr = np.sign(ur) * np.power(np.abs(ur), 0.2)
        print('\tw1: {}, w2: {}'.format(wl, wr))
        
        dx, dy, dth = self.dynamics(wl, wr)
        self._add_history()
        
        self.x += dx
        self.y += dy
        self.th += dth
        while self.th > 2 * np.pi:
            self.th -= 2 * np.pi
        while self.th < 0:
            self.th += 2 * np.pi


    def plot_history(self):
        nparr = np.asarray(self.history)
    
        X = list(nparr[:, 0])
        Y = list(nparr[:, 1])
        th = list(nparr[:, 2])
        t = list(nparr[:, 3])
    
        # colormap = plt.cm.YlGn(list(np.linspace(0.0, 1.0, 360)))
    
        plt.scatter(X, Y, c=t, cmap='RdBu')
        # plt.colorbar(colormap)
    
        plt.title('Position over Time')
        plt.xlabel('X (mm)')
        plt.ylabel('Y (mm)')
        
        mmin = np.minimum(np.min(X), np.min(Y))
        mmax = np.maximum(np.max(X), np.max(Y))

        # plt.xlim([0, mmax])
        # plt.ylim([0, mmax])
        # plt.plot([0, 750, 750, 0, 0], [0, 0, 500, 500, 0], c='k')
        
        plt.show()