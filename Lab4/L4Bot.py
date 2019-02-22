import numpy as np
import matplotlib.pyplot as plt


class Environment:
	def __init__(self, L, W, robL, robW):
		self.L = L
		self.W = W
		
		self.mar = max(robL, robW) / 2
	
		self.obstacles = []
		
	
	def load_obstacles(self, obs):
		"""
		
		:param obs: list of obstacles: obstacle is tuple of (x0, y0, length, width)
		:return:
		"""
		
		self.obstacles = np.asarray([o for o in obs])
		
		
	def plot_grid(self):
		# Plot edge
		x_edge = [0, self.L, self.L, 0, 0]
		y_edge = [0, 0, self.W, self.W, 0]
		plt.plot(x_edge, y_edge, c='k')
		
		# Plot edge margin
		x_edge = [self.mar, self.L-self.mar, self.L-self.mar, self.mar, self.mar]
		y_edge = [self.mar, self.mar, self.W-self.mar, self.W-self.mar, self.mar]
		plt.plot(x_edge, y_edge, c='r')

		for o in self.obstacles:
			x0 = o[0]
			y0 = o[1]
			
			l = o[2]
			w = o[3]
			
			# Plot obs
			x_obs = [x0, x0+l, x0+l, x0, x0]
			y_obs = [y0, y0, y0+w, y0+w, y0]
			plt.plot(x_obs, y_obs, c='g')
			
			# Plot margin
			x_mar = [x0-self.mar, x0+l+self.mar, x0+l+self.mar, x0-self.mar, x0-self.mar]
			y_mar = [y0-self.mar, y0-self.mar, y0+w+self.mar, y0+w+self.mar, y0-self.mar]
			plt.plot(x_mar, y_mar, c='r')

	def random_point(self):
		# return random point in environment
		x = numpy.random.uniform(low=0.0, high=self.L)
		y = numpy.random.uniform(low=0.0, high=self.W)
		return [x, y]
			

class L4Bot:
	def __init__(self, dimX, dimY, x0, y0, h0):
		self.x = x0
		self.y = y0
		self.h = h0
		self.i = 0
		self.history = []
		self.add_history()
		
		self.dimX = dimX
		self.dimY = dimY
	
		self.environment = Environment(dimX, dimY, 85, 90)
		self.Vertices = []
		self.Edges = []
		
	def add_history(self):
		
		self.history.append([self.x, self.y, self.h, self.i])
		self.i += 1
		
	
	def plot(self):
		self.environment.plot_grid()
		
		arr = np.asarray(self.history)
		x = arr[:, 0]
		y = arr[:, 1]
		h = arr[:, 2]
		t = arr[:, 3]
		
		plt.scatter(x, y, c=t, s=100, cmap='cool')
		plt.plot(x, y, c='b')
		
		plt.xlim((-0.5, self.dimX+0.5))
		plt.ylim((-0.5, self.dimY+0.5))
		plt.axis('equal')
		
		
		plt.show()


	def load_obstacles(self, obs):
		self.environment.load_obstacles(obs)

	def random_config(self):
		# returns random point + theta in config space
		x, y = self.Environment.random_point()
		theta = np.random.uniform(low=0.0,high=np.pi)
		return [x, y, theta]

	def drive(self, start, end):
		# drive in direction of end from start
		# To drive: rotate, drive, rotate. 

	def dist(self, start, end):
		# distance function, actually returns 'time'-like thing (but time proportional to dist)
		# I.e larger time required means it is "further", so it works out
		# Given we have to rotate, then drive forward, then rotate again (to match h)
		# draw line from start -> end, this angle is our intermediate_theta
		# TODO: find timeToTurnOneRadian, timeToTravelOneMM
		displacement_vector = [ end[0] - start[0], end[1] - start[1] ]
		intermediate_theta = np.arctan2(displacement_vector[1], displacement_vector[0])
		time_taken = abs(start[2] - intermediate_theta)*timeToTurnOneRadian
		time_taken += np.sqrt(displacement_vector[0]**2 + displacement_vector[1]**2)*timeToTravelOneMM
		time_taken += abs(end[2] - intermediate_theta)*timeToTurnOneRadian
		return time_taken
		
	def nearest_neighbour(self, point):
		# gets neareast neighbor of point (and distance)
		min_dist = self.dist(self.Vertices[0], point)
		neighbour = self.Vertices[0]
		for vertice in self.Vertices:
			d = self.dist(vertice, point)
			if d < min_dist:
				min_dist = d
				neighbour = vertice
		return neighbour, min_dist
			


		
