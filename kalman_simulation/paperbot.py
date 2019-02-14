import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

class PaperBot:
    """
    Paperbot Class.
    """
    def __init__(self, x0, y0, th0):
        # Initialize state, x, y, th
        self.x = x0
        self.y = y0
        self.th = th0
        
        # History of states for plotting
        self.history = []
        
        # Initialize measurements
        self.lidar_f = 0
        self.lidar_r = 0
        self.magnetometer = 0

        # Step number
        self.t = 0
        
        # Multiplicative constants
        self.dt = 0.1   # Time diff between updates
        self.d = 49     # Wheel diameter
        self.A = 84     # Axle length
        self.dconst = self.dt / 2 * np.pi * self.d

        self.dims = [750, 500]  # Box dimensions
        self.est_sensors = SensorEstimator(self.dims[0], self.dims[1])  # Estimates sensor measurements from state
        self._add_history()     # Add entry 0 to history (x0, y0, th0)
        
        # Covariances
        # Sensor cov
        self.R = np.zeros((3, 3))
        self.R[0, 0] = 3.28
        self.R[1, 1] = 1.86
        self.R[2, 2] = 0.8

        # TODO put correct values. I think these vals are right?
        # Actuators cov
        self.Q = np.zeros((3, 3))
        self.Q[0, 0] = 0.25 * self.dt
        self.Q[1, 1] = 0.25 * self.dt
        self.Q[2, 2] = 0.89
        
        # TODO initialize P properly
        # Kalman cov
        self.P = self.Q.copy()

        # Init jacobians to empty
        self.jacF = np.eye(3)
        self.jacH = np.eye(3)
    
    
    def _jacobian_H(self, min0=True, min1=True):    # Var to determine if min is l0 or l1
        """
        Populates jacobian of H given current state
        :param min0: bool: indicates if the minimum distance was the first (1/cos) or second (1/sin) for front lidar
        :param min1: bool: indicates if the minimum distance was the first (1/cos) or second (1/sin) for right lidar
        :return:
        """
        th0 = self.th
        th1 = self.th + np.pi / 2
        if th1 > 2 * np.pi:
            th1 -= 2 * np.pi
        
        min_arr = [min0, min1]
        
        # TODO fill up function. Ref is commented below
        for i, t in enumerate([th0, th1]):
            truth = min_arr[i]
            dy = self.dims[1] - self.y
            dx = self.dims[0] - self.x
            
            # Gradient with respect to theta (??)
            if t < np.pi / 2:
                if truth:
                    grad = dy * np.tan(t) / np.cos(t)
                else:
                    grad = dx * -1 / (np.sin(t) * np.tan(t))
            elif t < np.pi:
                if truth:
                    grad = dx
            elif t < 3 * np.pi / 2:
                pass
            else:
                pass
        
        # elif Th < self.p2 * 2:
        #     l0 = dx / coth
        #     l1 = Y / sith
        # elif Th < self.p2 * 3:
        #     l0 = Y / coth
        #     l1 = X / sith
        # else:
        #     l0 = X / coth
        #     l1 = dy / sith
        
        self.jacH[2, 2] = 1 / (1 + self.th ** 2)


    def _jacobian_F(self, wl, wr):
        """
        Populate jacobian of F given current state
        :param wl: angular velocity of left wheel
        :param wr: angular velocity of right wheel
        :return:
        """
        self.jacF[0, 0] = 1.
        self.jacF[0, 1] = 0.
        self.jacF[0, 2] = self.dconst * (wl + wr) * -self.ssin(self.th)
        print(self.jacF[0, 2])
        self.jacF[1, 0] = 0.
        self.jacF[1, 1] = 1.
        self.jacF[1, 2] = self.dconst * (wl + wr) * self.scos(self.th)
        
        self.jacF[2, 0] = 0.
        self.jacF[2, 1] = 0.
        self.jacF[2, 2] = 1.
        

    def _add_history(self):
        """
        Add an entry to  history arr and increase time_step t
        :return:
        """
        self.history.append([self.x, self.y, self.th, self.t])
        self.t += 1


    def scos(self, th):
        """
        Performs magentic cos. This is equiv to cos(90 - theta). Magnetometer is with respect to y and clockwise
        :param th: angle
        :return:
        """
        return np.cos(np.pi / 2 - th)


    def ssin(self, th):
        """
        Performs magentic sin. This is equiv to sin(90 - theta). Magnetometer is with respect to y and clockwise
        :param th: angle
        :return:
        """
        return np.sin(np.pi / 2 - th)


    def move(self, ul, ur):
        """
        Entry point for user. Simulate getting input ul and ur
        :param ul: int [0, 180]: input for left motor
        :param ur: int [0, 180]: input for right motor
        :return:
        """
        print('t: {} -- X: {}, Y: {}, Th: {}'.format(self.t, self.x, self.y, self.th))
        print('\tul: {}, ur: {}'.format(ul, ur))
        ul -= 90
        ur -= 90
    
        # Input to angular velocity
        wl = np.sign(ul) * np.power(np.abs(ul), 0.2)
        wr = np.sign(ur) * np.power(np.abs(ur), 0.2)
        print('\tw1: {}, w2: {}'.format(wl, wr))
    
        self.apriori(wl, wr)    # Apriori state update
        self.sense()            # Perform sensor measurements (simulate)
        #self.posteriori()       # Perform posteriori update
        self._add_history()     # Add current state
        
    def thetaToMag(self, th):
        mag_measure = 0
        th_deg = th * 180/np.pi
        if (th_deg < 190):
            mag_measure = 0.611*th_deg + 187
        elif (th_deg > 220):
            mag_measure = 0.876*th_deg + 250
        else:
            mag_measure = 4.41*th_deg - 529
        mag_measure = mag_measure * np.pi/180
        return mag_measure

    def distToLidarR(self, front_dist):
        return 0.983*front_dist+22.4

    def distToLidarF(self, right_dist):
        return 0.951*right_dist+47.4
    # Done? Check pls
    def sense(self):
        """
        Simulate lidar and magnetometer measurements, using numerics + some noise
        :return:
        """
        noise_th = np.random.normal(0, 2)
        noise_lidar_front = np.random.normal(0, 4)
        noise_lidar_right = np.random.normal(0, 4)
        
        #numerical approach, err=1, find point of intersection w walls and then get distance
        eps = 1
        xboundary, yboundary = self.dims
        #get right lidar with cos,sin (theta) 
        c, s = np.cos(self.th), np.sin(self.th)
        wallx = self.x
        wally = self.y
        while (wallx < xboundary and wallx > 0 and wally < yboundary and wally > 0):
            wallx = wallx +  c*eps
            wally = wally +  s*eps
        right_dist = dist([wallx, wally], [self.x, self.y]) + noise_lidar_right
        #get front lidar with cos,sin (theta+90)
        c, s = np.cos(self.th+np.pi/2), np.sin(self.th+np.pi/2)
        wallx = self.x
        wally = self.y
        while (wallx < xboundary and wallx > 0 and wally < yboundary and wally > 0):
            wallx = wallx + c*eps
            wally = wally + s*eps
        front_dist = dist([wallx, wally], [self.x, self.y]) + noise_lidar_front
        # these equations are our lab1 models
        # get magnetometer bearing based on theta
        
        self.lidar_f = self.distToLidarF(front_dist)
        self.lidar_r = self.distToLidarR(right_dist)
        self.magnetometer = self.thetaToMag(self.th)


    def apriori(self, wl, wr):
        """
        Apriori state update. Updates state and covariance given input
        :param wl: angular velocity of left wheel
        :param wr: angular velocity of right wheel
        :return:
        """
        
        # Movement equations
        dx =  self.dconst * self.scos(self.th) * (wl + wr)
        dy = self.dconst * self.ssin(self.th) * (wl + wr)
        dth = self.dconst / self.A * (wl - wr)

        self.x += dx
        self.y += dy
        self.th += dth

        while self.th > 2 * np.pi:
            self.th -= 2 * np.pi
        while self.th < 0:
            self.th += 2 * np.pi
        
        # Update JacobianF and Kalman cov
        self._jacobian_F(wl, wr)
        self.P = self.jacF.dot(self.P.dot(self.jacF.T)) + self.Q

        
    def posteriori(self):
        """
        Perform posteriori state update given sensor data
        :return:
        """
        # Get estimated sensor measurements from state
        estimates = self.est_sensors.ret(self.x, self.y, self.th)
        # TODO I changed what estimates returns to account for min(x/cos, y/sin) type of thing for gradients
        # TODO update accordingly
        estF = estimates[0][0]
		minF = 0
		if (estF < estimates[0][1]):
			estF = estimates[0][1]
			minF = 1
		
		estR = estimates[1][0]
        minR = 0
		if (estR < estimates[1][1]):
			estR = estimates[1][1]
			minR = 1
        # Calculate innovation and its cov
        innovation = np.asarray([self.lidar_f - est_F, self.lidar_r - est_R, self.magnetometer - est_M], float)
        innovation_cov = self.jacH.dot(self.P.dot(self.jacH.T)) + self.Q
                
		innovation_invertible = innovation_cov.copy()
        det = np.linalg.det(innovation_invertible)
		eps = 1e5
        while det == 0:
			innovation_invertible = innovation_cov + np.eye(3)*eps
			eps = eps/2
            det = np.linalg.det(innovation_invertible)

        Sinv = np.linalg.inv(innovation_invertible])        
        # TODO finish off kalman update
        kalmanGain = self.P.dot(self.jacH.T).dot(Sinv)
		#state update
		stateUpdate = kalmanGain.dot(innovation)
        self.x += stateUpdate[0]
		self.y += stateUpdate[1]
		self.th += stateUpdate[2]
		#cov update
		self.P =  (np.eye(3) - kalmanGain.dot(self.jacH)).dot(self.P)
        
    
    def plot_history(self):
        """
        Plots movement over time and final step lidar projections.
        Add any desired plottings here
        :return:
        """
        
        # Obtain x, y, th, t for every time t
        nparr = np.asarray(self.history)
    
        X = list(nparr[:, 0])
        Y = list(nparr[:, 1])
        Th = list(nparr[:, 2])
        t = list(nparr[:, 3])

        fig, ax = plt.subplots(nrows=1, ncols=1)
        
        ax.scatter(X, Y, c=t, cmap='RdBu')  # Scatter position over time
        
        # Text
        ax.set_title('Position over Time: {} - {}'.format(t[-1], Th[-1]))
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        
        # mmin = np.minimum(np.min(X), np.min(Y))
        # mmax = np.maximum(np.max(X), np.max(Y))
        # ax.xlim([0, mmax + 250])
        # ax.ylim([0, mmax + 250])
        
        ax.axis('equal')    # Maintain ratio
        ax.plot([0, self.dims[0], self.dims[0], 0, 0], [0, 0, self.dims[1], self.dims[1], 0], c='k')    # Plot box lines

        # Suplemental plotting
        self.est_sensors.plot_line(X[-1], Y[-1], Th[-1], ax)    # Add lidar projection lines
        ell = self.plot_ellipse()   # Covariance ellipse. Not currently working or ellipses are too big
        ax.add_artist(ell)  # Add ellipse to fig
        
        plt.show()

    
    def plot_ellipse(self):
        """
        Plot covariance ellipse centered at x, y
        :return:
        """
        cov = self.P[:2, :2]
        lambda_, v = np.linalg.eig(cov)
        lambda_ = np.sqrt(lambda_)
        
        ell = Ellipse(xy=(self.x, self.y),
                      width=lambda_[0] * 2, height=lambda_[1] * 2,
                      angle=np.rad2deg(np.arccos(v[0, 0])))
        ell.set_facecolor('none')
        return ell
        

class SensorEstimator:
    """
    SensorEstimator class. Estimates sensor measurements given state
    """
    def __init__(self, xdim, ydim):
        self.x = xdim
        self.y = ydim
        self.p = 2 * np.pi
        self.p2 = np.pi / 2


    def ret(self, X, Y, Th):
        """
        Returns sensor estimations given state
        :param X: current X coordinate
        :param Y: current Y coordinate
        :param Th: current state angle
        :return: list:
                    0 - front lidar [a/cos, b/sin] returns both line projections. Only take min. Both needed for gradient
                    1 - right lidar [a/cos, b/sin] returns both line projections. Only take min. Both needed for gradient
                    2 - Angle estimate (which is angle) but with added noise
        """
        #note we go from actual distance to lidar distance
        front_lidar_projections = [self.distToLidarR(d) for d in self.front_lidar(X, Y, Th) ]
        right_lidar_projections = [self.distToLidarF(d) for d in self.right_lidar(X, Y, Th) ] 
        return [front_lidar_projections, rightt_lidar_projections, self.magnetomer(Th)] 


    def front_lidar(self, X, Y, Th):
        """
        Predict front lidar distance reading from state
        :param X: current X coordinate
        :param Y: current Y coordinate
        :param Th: current state angle
        :return: both lidar projected distances. Only use min. First one is always a/cos and second is always b/sin
        """
        while Th > self.p:
            Th -= self.p
        while Th < 0:
            Th += self.p

        th0 = Th
        while th0 > self.p2:
            th0 -= self.p2

        coth = np.cos(th0) + 1e-7
        sith = np.sin(th0) + 1e-7
        
        dy = self.y - Y
        dx = self.x - X

        if Th < self.p2 * 1:
            l0 = dy / coth
            l1 = dx / sith
        elif Th < self.p2 * 2:
            l0 = dx / coth
            l1 = Y / sith
        elif Th < self.p2 * 3:
            l0 = Y / coth
            l1 = X / sith
        else:
            l0 = X / coth
            l1 = dy / sith

        return l0, l1


    def right_lidar(self, X, Y, Th):
        """
        Right lidar projection. Is same formulation as front lidar but shifted 90 degrees
        :param X: current X coordinate
        :param Y: current Y coordinate
        :param Th: current state angle
        :return: both lidar projected distances (small and big). Only use min. First one is always a/cos and second is always b/sin
        """
        return self.front_lidar(X, Y, Th + self.p2)


    def plot_line(self, X, Y, Th, ax):
        """
        Plot projected lines to walls for a given state
        :param X: state X coordinate
        :param Y: state Y coordinate
        :param Th: state angle
        :param ax: plotting axes
        :return:
        """
        
        # Get lines and use minimum
        lf0, lf1 = self.front_lidar(X, Y, Th)
        lf = min(lf0, lf1)
        lr0, lr1 = self.right_lidar(X, Y, Th)
        lr = min(lr0, lr1)

        # Get endpoints
        p21X = X + np.cos(self.p2 - Th) * lf
        p21Y = Y + np.sin(self.p2 - Th) * lf

        p22X = X + np.cos(-Th) * lr
        p22Y = Y + np.sin(-Th) * lr

        # Print endpoints
        ax.plot([X, p21X], [Y, p21Y],c='r', label='Front')
        ax.plot([X, p22X], [Y, p22Y],c='b', label='Right')
        ax.legend()
