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

        self.dims = [750, 500]
        self.sensors = SensorEstimator(self.dims[0], self.dims[1])


    def _add_history(self):
        self.history.append([self.x, self.y, self.th, self.t])
        self.t += 1


    def scos(self, th):
        return np.cos(np.pi / 2 - th)

    def ssin(self, th):
        return np.sin(np.pi / 2 - th)


    def dynamics(self, wl, wr):
        dconst = self.dt / 2 * np.pi * self.d
        dx =  dconst * self.scos(self.th) * (wl + wr)
        dy = dconst * self.ssin(self.th) * (wl + wr)
        dth = dconst / self.A * (wl - wr)
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
        Th = list(nparr[:, 2])
        t = list(nparr[:, 3])
    
        # colormap = plt.cm.YlGn(list(np.linspace(0.0, 1.0, 360)))
    
        plt.scatter(X, Y, c=t, cmap='RdBu')
        # plt.colorbar(colormap)
    
        plt.title('Position over Time: {} - {}'.format(t[-1], Th[-1]))
        plt.xlabel('X (mm)')
        plt.ylabel('Y (mm)')
        
        mmin = np.minimum(np.min(X), np.min(Y))
        mmax = np.maximum(np.max(X), np.max(Y))

        # plt.xlim([0, mmax + 250])
        # plt.ylim([0, mmax + 250])
        plt.axis('equal')
        plt.plot([0, self.dims[0], self.dims[0], 0, 0], [0, 0, self.dims[1], self.dims[1], 0], c='k')

        self.sensors.plot_line(X[-1], Y[-1], Th[-1])

        plt.show()


class SensorEstimator:
    def __init__(self, xdim, ydim):
        self.x = xdim
        self.y = ydim
        self.p = 2 * np.pi
        self.p2 = np.pi / 2


    def front_lidar(self, X, Y, Th):
        while Th > self.p:
            Th -= self.p
        while Th < self.p:
            Th += self.p

        th0 = Th
        while th0 > self.p2:
            th0 -= self.p2

        coth = np.cos(th0) + 1e-7
        sith = np.sin(th0) + 1e-7

        dy = self.y - Y
        dx = self.x - X


        if Th < self.p2 * 1:
            l = np.minimum(dy / coth, dx / sith)
        elif Th < self.p2 * 2:
            l = np.minimum(dx / coth, Y / sith)
        elif Th < self.p2 * 3:
            l = np.minimum(Y / coth, X / sith)
        else:
            l = np.minimum(dy / coth, X / sith)

        return l


    def right_lidar(self, X, Y, Th):
        return self.front_lidar(X, Y, Th + self.p2)


    def plot_line(self, X, Y, Th):
        lf = self.front_lidar(X, Y, Th)
        lr = self.right_lidar(X, Y, Th)
        print('~~', lf, lr)
        p21X = X + np.cos(self.p2 - Th) * lf
        p21Y = Y + np.sin(self.p2 - Th) * lf
        print(p21X, p21Y)

        p22X = X + np.cos(-Th) * lr
        p22Y = Y + np.sin(-Th) * lr
        print(p22X, p22Y)

        plt.plot([X, p21X], [Y, p21Y],c='r', label='Front')
        plt.plot([X, p22X], [Y, p22Y],c='b', label='Right')
        plt.legend()
