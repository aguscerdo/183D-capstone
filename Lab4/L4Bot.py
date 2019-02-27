import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.pyplot as plt
import time


# TODO: change these
timeToTurnOneRadian = 1.0 # 1000 ms
timeToTravelOneMM = 0.01 # 10 ms
xconst = 1.0/360/timeToTravelOneMM 
yconst = 1.0/360/timeToTravelOneMM
thconst = 1.0/180/timeToTurnOneRadian

RightTurn = [0.0, 180.0]
LeftTurn = [180.0,0.0]
ForwardMovement = [180.0, 180.0]

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
		x = np.random.uniform(low=0.0, high=self.L)
		y = np.random.uniform(low=0.0, high=self.W)
		return [x, y]
			
	def collision(self, state):
		# return True if collission, False otherwise
		m = self.mar
		x, y = state[0], state[1]
		for obj in self.obstacles:
			lower_x_bound = obj[0]-m
			lower_y_bound = obj[1]-m
			upper_x_bound = lower_x_bound+obj[2]+m*2
			upper_y_bound = lower_y_bound+obj[3]+m*2
			if (lower_x_bound < x and x < upper_x_bound and lower_y_bound < y and y < upper_y_bound):
				return True
		return False


	def plot_3d_obstacles(self, ax):
		for obstacle in self.obstacles:
			x0 = obstacle[0]
			y0 = obstacle[1]
			l = obstacle[2]
			w  = obstacle[3]
			
			x_mar = np.asarray([
				x0-self.mar, x0-self.mar, x0-self.mar,
				x0+l+self.mar, x0+l+self.mar, x0+l+self.mar,
				x0+l+self.mar, x0+l+self.mar, x0+l+self.mar,
				x0-self.mar, x0-self.mar, x0-self.mar,
				x0-self.mar, x0-self.mar, x0-self.mar,
			])
			
			y_mar = np.asarray([
				y0-self.mar, y0-self.mar, y0-self.mar,
				y0-self.mar, y0-self.mar, y0-self.mar,
				y0+w+self.mar, y0+w+self.mar, y0+w+self.mar,
				y0+w+self.mar, y0+w+self.mar, y0+w+self.mar,
				y0-self.mar, y0-self.mar, y0-self.mar
			])
			
			x_mar = np.concatenate((x_mar, x_mar))
			y_mar = np.concatenate((y_mar, y_mar))
			
			h_mar = np.asarray([
				0, 1, 0,
				0, 1, 0,
				0, 1, 0,
				0, 1, 0,
				0, 1, 0,
				
			], dtype=np.float)
			
			h_mar = np.append(h_mar, (1-h_mar))
			h_mar *= 2*np.pi
			
			ax.plot(x_mar, y_mar, h_mar, c='k')

	def plot_2d_obstacles(self):
		for obstacle in self.obstacles:
			x0 = obstacle[0]
			y0 = obstacle[1]
			l = obstacle[2]
			w  = obstacle[3]
			
			x_mar = np.asarray([
				x0-self.mar, x0-self.mar, x0-self.mar,
				x0+l+self.mar, x0+l+self.mar, x0+l+self.mar,
				x0+l+self.mar, x0+l+self.mar, x0+l+self.mar,
				x0-self.mar, x0-self.mar, x0-self.mar,
				x0-self.mar, x0-self.mar, x0-self.mar,
			])
			
			y_mar = np.asarray([
				y0-self.mar, y0-self.mar, y0-self.mar,
				y0-self.mar, y0-self.mar, y0-self.mar,
				y0+w+self.mar, y0+w+self.mar, y0+w+self.mar,
				y0+w+self.mar, y0+w+self.mar, y0+w+self.mar,
				y0-self.mar, y0-self.mar, y0-self.mar
			])
			
			x_mar = np.concatenate((x_mar, x_mar))
			y_mar = np.concatenate((y_mar, y_mar))
			
			plt.plot(x_mar, y_mar, c='k')	

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
		self.vertices = []
		self.edges = []
		
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
		x, y = self.environment.random_point()
		theta = np.random.uniform(low=0.0,high=2*np.pi)
		return [x, y, theta]


	def move(self, start, action, t):
		# gets next state from start given action and time t
		th = start[2]
		wl = (action[0]-90.0)
		wr = (action[1]-90.0)
		dx =  1.0*t*xconst * np.cos(th) * (wl + wr)
		dy = 1.0*t*yconst * np.sin(th) * (wl + wr)
		dth = 1.0*t*thconst * (wl - wr)
		new_state = 1.0*np.copy(start)
		new_state[0] += dx
		new_state[1] += dy
		th += dth
		while th > 2 * np.pi:
			th -= 2 * np.pi
		while th < 0:
			th += 2 * np.pi
		new_state[2] = th
		return new_state

	def drive(self, start, actions, t=2, step=0.1):
		# drive using actions from start, for t seconds,
		# step size tells us how many vertices/edges to add to our graph
		state = np.copy(start)
		next_state = np.copy(start)
		i = 0
		num_actions = len(actions)
		curr_time = 0.0
		# time taken for first action
		threshold = actions[0][2]
		while(i < num_actions and curr_time < t):
			# check if we need to go to new action
			if (curr_time > threshold):
				i = i + 1
				if (i < num_actions):
					threshold += actions[i][2]
				else: 
					return state
			# get next state and grow tree, if no collision
			next_state = self.move(state, actions[i][0], step)
			if (self.environment.collision(next_state)):
				return None
			curr_time += step
			# we dont add here in original RRT, i think we should(?)
			#self.vertices.append(next_state) 
			#self.edges.append([state, next_state])
			state = np.copy(next_state)
		return state

	def turn(self, start, end):
		# finds direction and amount to turn to get from start to end
		# direction = -1 -> left, +1 ->right
		if ( end - start < np.pi and end - start >= 0):
			direction = -1
			amount = end - start
		elif (end - start >= np.pi and end - start >= 0):
			direction = 1
			amount = 2*np.pi - (end - start)
		elif ( start - end < np.pi and start - end >= 0):
			direction = 1
			amount = start - end
		else: 
			direction = -1
			amount = 2*np.pi - (start - end)
		amount = abs(amount)
		return amount, direction
			
	def dist(self, start, end):
		# distance function, actually returns 'time'-like thing (but time proportional to dist)
		# I.e larger time required means it is "further", so it works out
		# Given we have to rotate, then drive forward, then rotate again (to match h)
		# draw line from start -> end, this angle is our intermediate_theta
		# TODO: find timeToTurnOneRadian, timeToTravelOneMM
		actions = []
		time_taken = 0
		displacement_vector = [ end[0] - start[0], end[1] - start[1] ]
		intermediate_theta = np.arctan2(displacement_vector[1], displacement_vector[0])
		amount, direction = self.turn(start[2], intermediate_theta)
		t1 = amount*timeToTurnOneRadian
		if (direction == -1):
			actions.append([LeftTurn, amount, t1])
		else:
			actions.append([RightTurn, amount, t1])
		amount = np.sqrt(displacement_vector[0]**2 + displacement_vector[1]**2)
		t2 = amount*timeToTravelOneMM
		actions.append([ForwardMovement, amount, t2])
		amount, direction = self.turn(intermediate_theta, end[2])
		t3 = amount*timeToTurnOneRadian
		if (direction == -1):
			actions.append([LeftTurn, amount, t3])
		else:
			actions.append([RightTurn, amount, t3])
		time_taken = t1 + t2 + t3
		return time_taken, actions
		
	def nearest_neighbour(self, point):
		# gets neareast neighbor of point (and distance)
		action_set = []
		min_dist, action_set = self.dist(self.vertices[0], point)
		neighbour = self.vertices[0]
		for vertice in self.vertices:
			d, actions = self.dist(vertice, point)
			if d < min_dist:
				min_dist = d
				neighbour = vertice
				action_set = actions
		return neighbour, actions, min_dist
			
	def RRT(self, start_state=None, num_branches=10):
		if (start_state is None):
			start_state = [self.x, self.y, self.h]
		self.vertices.append(start_state)
		for k in range(num_branches):
			randpt = self.random_config()
			neighbor, actions, dist = self.nearest_neighbour(randpt)
			new_state = self.drive(neighbor, actions, t=1)
			if new_state is not None:
				self.vertices.append(new_state)
				self.edges.append([neighbor, new_state])
	
	def visualise_RRT(self):
		for edge in self.edges:
			xs = [edge[i][0] for i in range(2)]
			ys = [edge[i][1] for i in range(2)]
			plt.plot(xs, ys, 'r')
		plt.show()
		
		fig = plt.figure()
		ax = p3.Axes3D(fig)
		for edge in self.edges:
			v1, v2 = edge
			x1, y1, th1 = v1
			x2, y2, th2 = v2
			ax.plot([x1, x2], [y1, y2], zs=[th1, th2], color='red')
		
		self.environment.plot_3d_obstacles(ax)
		plt.show()

	def plot_path(self, path, start, goal, v_initial, v_final):
		fig = plt.figure()
		for edge in self.edges:
			xs = [edge[i][0] for i in range(2)]
			ys = [edge[i][1] for i in range(2)]
			plt.plot(xs, ys, 'r')
		for edge in path:
			xs = [edge[i][0] for i in range(2)]
			ys = [edge[i][1] for i in range(2)]
			plt.plot(xs, ys, 'g')
		plt.scatter(start[0], start[1], c='b', s=50, alpha=0.8)
		plt.scatter(goal[0], goal[1], c='y', s=50, alpha=0.8)
		plt.scatter(v_initial[0], v_initial[1], c='c', s=50, alpha=0.8)
		plt.scatter(v_final[0], v_final[1], c='k', s=50, alpha=0.8)
		self.environment.plot_2d_obstacles()
		plt.show()

	

	def findPath(self, start=None, goal=None):
		
		if (start is None):
			start = [self.x, self.y, self.h]
		if (goal is None):
			goal = self.random_config()
		print("Finding path: " + str(start) + "-->" + str(goal))
		# find nearest neighbors
		v_initial, _, _ = self.nearest_neighbour(start)
		v_final, _, _ = self.nearest_neighbour(goal)
		print("Finding path: " + str(v_initial) + "-->" + str(v_final))
		# find path via depth-first search
		visited = [[v,np.array_equal(v, v_initial)] for v in self.vertices]
		curr = np.copy(v_initial)
		v_next = []
		path = []
		def visit(vertice):
			for i in range(len(visited)):
				if ( np.array_equal(visited[i][0], vertice)):
					if (visited[i][1]):
						return True
					else:
						visited[i][1] = True
						return False
			print("problem!>")
		n2 = len(visited)
		stopCondition = False
		while not stopCondition:
			pop = True
			#print("step")
			#print("curr: " + str(curr))
			#print("Num visited: " + str(N))
			#print("Num not visited: " + str(n2))
			for e in self.edges:
				if np.array_equal(e[0], curr):
					v_next = e[1]
				elif np.array_equal(e[1], curr):
					v_next = e[0]
				else:
					v_next = None
				if (v_next is not None and not visit(v_next)):
					n2 -= 1
					#print("next: " + str(v_next))
					path.append([curr, v_next])
					curr = np.copy(v_next)
					if (np.array_equal(curr, v_final)):
						stopCondition = True
					pop = False
					break
			if (pop):
				#print("pop!")
				curr = path[-1][0]
				path = path[:-1]
				
		self.plot_path(path, start, goal, v_initial, v_final)
		return path
			

					
			
			


		
			

			
