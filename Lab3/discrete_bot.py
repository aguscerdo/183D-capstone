import matplotlib.pyplot as plt
import numpy as np
import sys
import time

class DiscreteBot:
	def __init__(self, W, L, x0=0, y0=0, h0=0, p_error=0.0):
		# Initialize states
		self.L = L
		self.W = W
		
		self.x = x0
		self.y = y0
		self.h = h0
		self.history = []
		
		self.p_error=p_error
		
		# 2.1.a
		self.S = np.zeros((L, W))   # N_s = W * L
		self.build_state_grid()
		
		self.policy_grid = np.zeros((L, W, 12, 2))
		self.build_policy_grid()
		self.goal = (4,4,-1)
		
		self.value_grid = np.ones((L, W, 12)) * (-sys.maxsize - 1)
		self.next_value_grid = np.ones((L, W, 12)) * (-sys.maxsize - 1)
		self.lookahead_grid = np.zeros_like(self.policy_grid)

	
	def add_history(self):
		self.history.append([self.x, self.y, self.h])
	
	
	def build_state_grid(self):
		"""
		2.2.a: Build grid values
		:return:
		"""
		for i in range(self.L):
			self.S[i, 0] = -100
			self.S[i, self.W-1] = -100
		for i in range(self.W):
			self.S[0, i] = -100
			self.S[self.L-1, i] = -100
		
		batch = [[3, 3, -10], [3, 4, -10], [-2, -2, 1]]
		self.__state_add_batch(batch)
	
	
	def __state_add_batch(self, batch):
		for b in batch:
			self.S[b[0], b[1]] = b[2]
	
	
	# ~~~~~~ Helpers for movement and prediction ~~~~~~~~~
	
	@staticmethod
	def heading_to_direction(h_s):
		"""
		Transforms a heading into quadrant and edge
		
		Quadrants: 0, +Y | 1, +X | 2, -Y | 3, -X
		Edges: -1, one left of axis | 0, on axis | +1, one right of axis
		
		e.g.    10 -> Q3, E1
				3 -> Q1, E0
				4 -> Q1, E1
				5 -> Q2, E-1
		
		:param h_s: heading
		:return:
		"""
		h = h_s
		if h in [11, 0, 1]:
			ret0 = 0
		elif h in [2, 3, 4]:
			ret0 = 1
		elif h in [5, 6, 7]:
			ret0 = 2
		else:
			ret0 = 3
		
		return ret0, (h+1) % 3
		
	
	def update_x(self, diff):
		if 0 <= self.x + diff < self.L:
			self.x = int(self.x+diff)
			return True
		else:
			return False
	
	
	def update_y(self, diff):
		if 0 <= self.y + diff < self.W:
			self.y = int(self.y+diff)
			return True
		else:
			return False
	
	
	def update_h(self, h):
		if h < 0:
			h += 12
		self.h = int(h % 12)
	
	
	def move(self, movement, turning):
		"""
		2.1.b
		Action Space Options:
		- Do nothing, 1
		- Turning, 3 (left, right, inplace)
		- Move, 2 (Front, Back)

		N_a = 1 + 3*2

		:param movement: (int) -1 backwards, 0 in place, +1 front
		:param turning: (int) -1 turn left, 0 in place, +1 right
		:return:
		"""
		if movement == 0:
			return
		
		err = np.random.uniform(0, 1)
		if err < self.p_error:
			# Turn -1
			self.update_h(self.h - 1)
		elif err < self.p_error * 2:
			# Turn +1
			self.update_h(self.h + 1)
		
		h, _ = self.heading_to_direction(self.h)
		
		if h == 0:
			wall = self.update_y(movement * 1)
		elif h == 1:
			wall = self.update_x(movement * 1)
		elif h == 2:
			wall = self.update_y(movement * -1)
		else:
			wall = self.update_x(movement * -1)
		
		self.update_h(self.h + turning)
	
	
	@staticmethod
	def state_difference(x_s, y_s, h_s, x_target, y_target, h_target):
		return x_target - x_s, y_target - y_s, h_target - h_s
	
	
	def move_probability(self, x_s, y_s, h_s, movement, turning, x_target, y_target, h_target):
		"""
		2.1.c: Receives state and input and returns probability
		
		:param x_s: initial state x
		:param y_s: initial state y
		:param h_s:	initial state h
		:param movement: movement direction (-1, +1)
		:param turning: heading turning (-1, 0, +1)
		:param x_target: target x position
		:param y_target: target y position
		:param h_target: target h position
		:return: (float) probability of going to s' given the action taken
		"""
		if not (0 <= x_target < self.L and 0 <= y_target < self.W):
			return 0.
		
		if h_s > 12:
			h_s %= 12
		while h_s < 0:
			h_s += 12
	
		dx, dy, dh = self.state_difference(x_s, y_s, h_s, x_target, y_target, h_target)
		
		if np.abs(dx) + np.abs(dy) != 1:
			# Too far or in-place
			return 0.
		if np.abs(dh) > 2:
			return 0.
		
		h_dir, h_edge = self.heading_to_direction(h_s)
		
		dx *= movement
		dy *= movement
		dh = dh
		ret = 0
		
		if dx == 1:
			# Heading direction is along axis
			if h_dir == 1:
				if h_edge == 1: # Facing forward so will move to next block
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=0)
				else:
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=1)
					
			elif (h_dir == 0 and h_edge == 2) or (h_dir == 2 and h_edge == 0):
				ret = self.__prob_heading_helper(turning, dh, h_edge, mode=2)

		
		elif dx == -1:
			if h_dir == 3:
				if h_edge == 1:
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=0)
				else:
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=1)
					
			elif (h_dir == 2 and h_edge == 2) or (h_dir == 0 and h_edge == 0):
				ret = self.__prob_heading_helper(turning, dh, h_edge, mode=2)

		
		elif dy == 1:
			if h_dir == 0:
				if h_edge == 1:
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=0)
				else:
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=1)
			elif (h_dir == 1 and h_edge == 0) or (h_dir == 3 and h_edge == 2):
				ret = self.__prob_heading_helper(turning, dh, h_edge, mode=2)
		
		elif dy == -1:
			if h_dir == 2:
				if h_edge == 1:
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=0)
				else:
					ret = self.__prob_heading_helper(turning, dh, h_edge, mode=1)
			elif (h_dir == 1 and h_edge == 2) or (h_dir == 3 and h_edge == 0):
				ret = self.__prob_heading_helper(turning, dh, h_edge, mode=2)
		
		return float(ret)
	
	def __prob_heading_helper(self, turning, dh, heading_edge, mode=0):
		"""
		Helper with lots of if statements
		:param turning: turning input
		:param dh: difference from current to target heading
		:param heading_edge: which edge of the heading is it on (0, 1, 2) -> (left, center, right)
		:param mode: mode to run on (0, 1, 2) ->
						on central heading moving forward, on edge moving forward, on other heading edge moving forward)
		:return: probability
		"""
		dh_abs = np.abs(dh)
		ret = 0
		p_err = self.p_error
		comp_err = 1 - 2 * self.p_error
		
		
		# Heading is straight up
		if mode == 0:
			if turning == 0:
				ret = p_err * (dh_abs == 1) + comp_err * (dh == 0)
				
			elif turning == -1:
				ret = p_err * (dh == -2 or dh == 0) + comp_err * (dh == -1)
			elif turning == 1:
				ret = p_err * (dh == -2 or dh == 0) + comp_err * (dh == 1)
		
		# Heading is an edge <-1  +1> but want to forward block
		elif mode == 1:
			if turning == 0:  # No turning
				if heading_edge == 0:  # Left edge
					ret = comp_err * (dh == 0) + p_err * (dh == 1)
				
				elif heading_edge == 2:
					ret = comp_err * (dh == 0) + p_err * (dh == -1)
			
			elif turning == 1:
				if heading_edge == 0:
					ret = comp_err * (dh == 1) + p_err * (dh == 2)
				
				if heading_edge == 2:
					ret = comp_err * (dh == 1) + p_err * (dh == 0)
			
			elif turning == -1:
				if heading_edge == 0:
					ret = comp_err * (dh == -1) + p_err * (dh == 0)
				elif heading_edge == 2:
					ret = comp_err * (dh == -1) + p_err * (dh == -2)
		
		elif mode == 2:  # On an edge of another heading
			if turning == 0:
				if (heading_edge == 2 and dh == 1) or (heading_edge == 0 and dh == -1):
					ret = self.p_error
			
			elif turning == 1:
				if (heading_edge == 2 and dh == 2) or (heading_edge == 0 and dh == 0):
					ret = self.p_error
			
			elif turning == -1:
				if (heading_edge == 2 and dh == 0) or (heading_edge == 0 and dh == -2):
					ret = self.p_error
		
		return float(ret)
	
	def next_state(self, x_s, y_s, h_s, movement, turning):
		"""
		2.1.d
		Returns most probable state given state and input over all adjac
		:param x_s: initial state x
		:param y_s: initial state y
		:param h_s: initial state h
		:param movement: movement direction (-1, +1)
		:param turning: turning direction (-1, 0, +1)
		:return: (x, y, h) next most probable state
		"""
		# if at edge or move = 0, no error
		if movement == 0:
			return [x_s, y_s, h_s]
		elif not (0<= x_s < self.L and 0<= y_s < self.W):
			nh = h_s + next_state[2] - 2
			if nh < 0: nh += 12
			nh = nh % 12
			return [x_s, y_s, nh]
		
		if h_s > 12:
			h_s = h_s % 12
		while h_s < 0:
			h_s += 12
		
		prob = np.zeros((3, 3, 5))
		
		for di in [-1, 0, 1]:
			for dj in [-1, 0, 1]:
				for dh in [-2, -1, 0, 1, 2] :
					if di == dj == 0 or np.abs(di) + np.abs(dj) > 1:
						continue
					
					prob[di+1, dj+1, dh+2] = self.move_probability(x_s, y_s, h_s,
							movement, turning, x_s+di, y_s+dj, h_s+dh)
					
		next_state = np.unravel_index(np.argmax(prob), prob.shape)
		nx = x_s + next_state[0] - 1
		if nx < 0:
			nx = 0
		elif nx >= self.L:
			nx = self.L-1
	
		ny = y_s + next_state[1] - 1
		if ny < 0:
			ny = 0
		elif ny >= self.W:
			ny = self.W-1
			
		nh = h_s + next_state[2] - 2
		if nh < 0: nh += 12
		nh = nh % 12
		
		return nx, ny, nh
	
	
		
	
	def reward(self, x_s, y_s, h_s=-1):
		"""
		2.2.a
		:param x_s: state x position
		:param y_s: state y position
		:return:
		"""
		if (h_s == -1 or self.goal[2] == -1):
			# don't care about h, reward is same
			return self.S[x_s, y_s]
		else:
			# 2.5, care about h 
			# if state is goal state, then reward if h == 6
			if x_s == self.goal[0] and y_s == self.goal[1] and self.goal[2] >= 0:
				return (h_s == 6) * self.S[x_s, y_s]
		# if state not goal state, reward is the same
		return self.S[x_s, y_s]
	

	# TODO policy gridding
	def build_policy_grid(self):
		"""
		2.3.a
		:return:
		"""
		goal = (4, 4)
		#new policy, we always turn
		def greedy_policy(state):
			xs = state[0]
			ys = state[1]
			hs = state[2]
			#Quadrants: 0, +Y | 1, +X | 2, -Y | 3, -X
			direction, edge = self.heading_to_direction(hs)
			# get move direction, -1 => back, 1 => fwd
			# get turn direction, -1 => left, 1 => right, 0 => don't turn
			moveDirection = 0
			turnDirection = 0
			if direction == 0:
				#+y, 
				#if goal has greater or same y then move forward else backward
				moveDirection = 1*(goal[1] >= ys) + -1*(goal[1] < ys)
				#if goal to "left", then we have greater x coord
				turnDirection = -1*(goal[0] < xs) + 1*(goal[0] > xs)
			elif direction == 1:
				#+x, 
				#if goal has greater or same x then move forward else backward
				moveDirection = 1*(goal[0] >= xs) + -1*(goal[0] < xs)
				#if goal to "left", then we have lesser y coord
				turnDirection = -1*(goal[1] > ys) + 1*(goal[1] < ys)
			elif direction == 2:
				#-y, 
				#if goal has lesser or same y then move forward else backward
				moveDirection = 1*(goal[1] <= ys) + -1*(goal[1] > ys)
				#if goal to "left", then we have lesser x coord
				turnDirection = -1*(goal[0] > xs) + 1*(goal[0] < xs)
			else:
				#-x, 
				#if goal has lesser or same x then move forward else backward
				moveDirection = 1*(goal[0] <= xs) + -1*(goal[0] > xs)
				#if goal to "left", then we have greater y coord
				turnDirection = -1*(goal[1] < ys) + 1*(goal[1] > ys)
			return [moveDirection, turnDirection]
		
		for i in range(self.L):
			for j in range(self.W):
				for h in range(12):
					state = [i, j, h]
					action = greedy_policy(state)
					self.policy_grid[i, j, h, :] = action
	
	
	def simulate_trajectory(self, x0=None, y0=None, h0=None, p_error=None, goal=None, match_h=False):
		"""
		Initiates movement given policy grid.
		While the goal has not been reached, take the appropriate policy action, move according to it and add to history


		:param x0: initial state x
		:param y0: initial state y
		:param h0: initial state h
		:param p_error: probability of error
		:param goal: target x, y
		:param match_h: match the heading as goal
		:return:
		"""
		if x0:
			self.x = x0
		if y0:
			self.y = y0
		if h0:
			self.h = h0
		if p_error:
			self.p_error = p_error
		if goal:
			self.goal = goal
			
		self.history = []
		self.add_history()
		
		while not (self.x == self.goal[0] and self.y == self.goal[1]) and not (match_h and self.h == self.goal[2]):
			mov = self.policy_grid[self.x, self.y, self.h]
			mov, turn = mov[0], mov[1]
			self.move(mov, turn)
			self.add_history()
			# self.plot_grid()

		# self.add_history()

		
	def plot_grid(self):
		"""
		Plots movement history grid
		:return:
		"""
		np_hist = np.asarray(self.history)
		xh = np_hist[:, 0] + 0.5
		yh = np_hist[:, 1] + 0.5
		hh = np_hist[:, 2]
		th = np.arange(0, np_hist.shape[0])
		
		plt.scatter(xh, yh, c=th, cmap='cool', s=200)
		plt.plot(xh, yh)
		plt.colorbar()
		
		plt.xlim([-1, self.L + 1])
		plt.ylim([-1, self.W + 1])
		
		lx = [0, self.L, self.L, 0, 0]
		ly = [0, 0, self.W, self.W, 0]
		plt.plot(lx, ly, c='k')
		
		s = 750 * 6 / self.L
		a = 0.25
		plt.scatter(self.goal[0]+0.5, self.goal[1]+0.5, alpha=a, s=s, c='g')

		for ii in range(self.L):
			for jj in range(self.W):
				el = self.S[ii, jj]

				if el < -10:
					plt.scatter(ii + 0.5, jj + 0.5, alpha=a, c='r', s=s)
				elif el < 0:
					plt.scatter(ii + 0.5, jj + 0.5, alpha=a, c='y', s=s)
				elif el > 0:
					plt.scatter(ii + 0.5, jj + 0.5, alpha=a, c='g', s=s)
		
		plt.grid(True, 'both', 'both')
		plt.title('Movement: {}'.format(th[-1]))
		plt.show()
	
	
	def run_23c(self):
		"""
		2.3.c run the given simulation
		:return:
		"""
		self.build_policy_grid()
		self.simulate_trajectory(x0=1, y0=4, h0=6, p_error=0, goal=(4, 4))
		self.plot_grid()

		
	def build_value_grid(self,discount_factor, policy_matrix=None):
		"""
		2.3.d
		Returns a value matrix given a policy
		:param discount_factor: lambda to discount horizon
		:param policy_matrix: policy matrix to use. If none, use self matrix
		:return:
		"""
		if policy_matrix is not None:
			self.policy_grid = policy_matrix
			
		# Initialize to -INT (-infinity)
		self.value_grid = np.ones((self.L, self.W, 12)) * (-sys.maxsize - 1)
		mmin = (-sys.maxsize - 1)
		
		def recursive_value(xs, ys, hs, rec_hist):
			if xs == self.goal[0] and ys == self.goal[1]:
				if (self.goal[2] > -1):
					return self.reward(xs, ys, hs) * 1.0/(1-discount_factor)
				return self.reward(xs, ys) * 1.0/(1-discount_factor) 
			if self.value_grid[xs, ys, hs] > mmin:
				return self.value_grid[xs, ys, hs]
			
			for hh in rec_hist:
				if hh[0] == xs and hh[1] == ys and hh[2] == hs:
					return self.reward(xs, ys)
			# make sure we are not going to be in an inf loop
			N = min(len(rec_hist), 100)
			if (N == 100):
				print ("Cap reached! may need to increase N for optimal results")
			rec_hist = [[xs, ys, hs]] + rec_hist[:N]
			
			mov = self.policy_grid[xs, ys, hs]
			mov, turn = mov[0], mov[1]
			
			xs2, ys2, hs2 = self.next_state(xs, ys, hs, mov, turn)
			
			# print(xs, ys, hs, '~~~',  xs2, ys2, hs2, '~~~', mov, turn)
			if xs2 == xs and ys2 == ys and hs2 == hs:
				return self.reward(xs, ys)  * 1.0/(1-discount_factor)
			
			return self.reward(xs, ys) + discount_factor * recursive_value(xs2, ys2, hs2, rec_hist)
		
		for x in range(self.L):
			for y in range(self.W):
				for h in range(12):
					self.value_grid[x, y, h] = recursive_value(x, y, h, [])

		return self.value_grid
			
	
	def run_23e(self):
		"""
		2.3.e
		:return:
		"""
		self.build_state_grid()
		self.build_policy_grid()
		self.build_value_grid(0.9)
		x, y, h = 1, 4, 6
		value = self.value_grid[x, y, h]
		print('Cost of {}, {}, {}: {}'.format(x, y, h, value))


	def build_lookahead_grid(self,value_function, discount_factor):
		# 2.3.f
		# policy(s)= argmax_a (sum_s' p(s,a,s')*(r(s')+discount*V(s')) )
		def onesteplookahead(state):
			# value_by_action[i][j] = value if action_ij taken
			# i=0 -> backward=-1, i=1 -> forward=1
			# j=0 -> left=-1, j=1 -> no turn=0, j=2-> right=1
			value_by_action = np.zeros([2,3])
			val_plus_reward = discount_factor * value_function 
			for i in range(self.L):
				for j in range(self.W):
					for k in range(12):
						val_plus_reward[i, j, k] += self.reward(i, j, k)
			bestAction = [0, 0]
			# if dont move, reward is 
			action_mov = 0
			action_turn = 0
			bestVal = 0
			for ii in range(self.L):
						for jj in range(self.W):
							for hh in range(12):
								bestVal += self.move_probability(state[0], state[1], state[2], action_mov, action_turn, ii, jj, hh) \
								                        * val_plus_reward[ii, jj, hh]
			# sum_s' p(s,a,s') * val_plus_reward
			for i in range(2):
				for j in range(3):
					value_by_action[i, j] = 0 
					action_mov = 2*i-1 # i=0 -> backward=-1, i=1 -> forward=1
					action_turn = j-1
					# now loop over all states s'
					for ii in range(self.L):
						for jj in range(self.W):
							for hh in range(12):
								value_by_action[i,j] += self.move_probability(state[0], state[1], state[2],
								                                              action_mov, action_turn, ii, jj, hh) \
								                        * val_plus_reward[ii, jj, hh]
					if (value_by_action[i, j] > bestVal):
						bestVal = value_by_action[i, j]
						bestAction = action_mov, action_turn
			return bestAction
		
		for i in range(self.L):
			for j in range(self.W):
				for h in range(12):
					c_state = [i, j, h]
					self.lookahead_grid[i, j, h, :] = onesteplookahead(c_state)
		
	
	def policy_iteration(self, discount_factor, x0=None, y0=None, h0=None, p_error=None, goal=None, match_h=False):
		"""
		2.3.g
		:return:
		"""
		self.goal = goal # justin case
		self.build_state_grid()
		self.build_policy_grid()
		self.build_value_grid(discount_factor)
		print('Lookahead: 0')
		#prev_hash = hash(self.lookahead_grid.tobytes())
		prev_grid = np.copy(self.lookahead_grid)
		self.build_lookahead_grid(self.value_grid, discount_factor)
		self.build_value_grid(discount_factor, self.lookahead_grid)
		#new_hash = hash(self.lookahead_grid.tobytes())
		new_grid = np.copy(self.lookahead_grid)
		i = 0
		#while not prev_hash == new_hash:
		stopCondition = (np.sum(np.not_equal(prev_grid, new_grid)) < 1)
		while not stopCondition:
			i += 1
			print('Lookahead: {}'.format(i))
			#prev_hash = new_hash
			prev_grid = np.copy(new_grid)
			self.build_lookahead_grid(self.value_grid, discount_factor)
			self.build_value_grid(discount_factor, self.lookahead_grid)
			#new_hash = hash(self.lookahead_grid.tobytes())
			new_grid = np.copy(self.lookahead_grid)
			differBy = np.sum(np.not_equal(prev_grid, new_grid))
			print("Differ by: " + str(differBy))
			stopCondition = (differBy < 1) or (i > 10 and differBy < 20) or (i > 25 and differBy < 50)

		if x0:
			self.x = x0
		if y0:
			self.y = y0
		if h0:
			self.h = h0
		if p_error:
			self.p_error = p_error
		if goal:
			self.goal = goal
		
		self.history = []
		self.add_history()
		print('MOVING')
		stopCondition = (self.x == self.goal[0]) and (self.y == self.goal[1]) and (self.h == self.goal[2] and match_h)
		while not stopCondition:
			mov = self.lookahead_grid[self.x, self.y, self.h]
			print('stepping')
			print (mov)
			mov, turn = mov[0], mov[1]
			self.move(mov, turn)
			self.add_history()
			stopCondition = (self.x == self.goal[0]) and (self.y == self.goal[1]) and (self.h == self.goal[2] and match_h) or (mov == 0)
			# self.plot_grid()

	
	
	def run_23h(self):
		self.policy_iteration(0.9, 1, 4, 6, 0.0, (4,4), False)
		self.plot_grid()
	
	def build_value_zero(self):
		# 2.4.a
		self.value_grid = np.zeros((self.L, self.W, 12))

	def build_next_value_grid(self,value_function, discount_factor):
		# 2.4.a
		# v^n(s) = max_a (sum_s' p(s,a,s')*(r(s')+discount*V^(n-1)(s')) )
		def next_value(state):
			# value_by_action[i][j] = value if action_ij taken
			# i=0 -> backward=-1, i=1 -> forward=1
			# j=0 -> left=-1, j=1 -> no turn=0, j=2-> right=1
			value_by_action = np.zeros([2,3])
			val_plus_reward = discount_factor * value_function 
			for i in range(self.L):
				for j in range(self.W):
					for k in range(12):
						val_plus_reward[i, j, k] += self.reward(i, j, k)
			bestAction = [0, 0]
			action_mov = 0
			action_turn = 0
			bestVal = 0
			# if dont move, reward is: 
			for ii in range(self.L):
						for jj in range(self.W):
							for hh in range(12):
								bestVal += self.move_probability(state[0], state[1], state[2], action_mov, action_turn, ii, jj, hh) \
								                        * val_plus_reward[ii, jj, hh]
			# sum_s' p(s,a,s') * val_plus_reward
			for i in range(2):
				for j in range(3):
					value_by_action[i, j] = 0 
					action_mov = 2*i-1 # i=0 -> backward=-1, i=1 -> forward=1
					action_turn = j-1
					# now loop over all states s'
					for ii in range(self.L):
						for jj in range(self.W):
							for hh in range(12):
								value_by_action[i,j] += self.move_probability(state[0], state[1], state[2],
								                                              action_mov, action_turn, ii, jj, hh) \
								                        * val_plus_reward[ii, jj, hh]
					if (value_by_action[i, j] > bestVal):
						bestVal = value_by_action[i, j]
						bestAction = action_mov, action_turn
			return bestVal
		
		for i in range(self.L):
			for j in range(self.W):
				for h in range(12):
					c_state = [i, j, h]
					self.next_value_grid[i, j, h] = next_value(c_state)
	
	def value_iteration(self, discount_factor, x0=None, y0=None, h0=None, p_error=None, goal=None, match_h=False):
		"""
		2.4.a
		:return:
		"""
		self.goal = goal # justin case
		self.build_state_grid()
		self.build_value_zero()
		print('V^0')
		#prev_hash = hash(self.lookahead_grid.tobytes())
		self.build_lookahead_grid(self.value_grid, discount_factor)
		prev_grid = np.copy(self.lookahead_grid)
		self.build_next_value_grid(self.value_grid, discount_factor)
		self.build_lookahead_grid(self.next_value_grid, discount_factor)
		new_grid = np.copy(self.lookahead_grid)
		#new_hash = hash(self.lookahead_grid.tobytes())
		
		i = 0
		#while not prev_hash == new_hash:
		while not np.array_equal(prev_grid, new_grid):
			i += 1
			print('V^{}'.format(i))
			#prev_hash = new_hash
			prev_grid = np.copy(new_grid)
			self.value_grid = np.copy(self.next_value_grid)
			self.build_next_value_grid(self.value_grid, discount_factor)
			self.build_lookahead_grid(self.next_value_grid, discount_factor)
			#new_hash = hash(self.lookahead_grid.tobytes())
			new_grid = np.copy(self.lookahead_grid)
		
		self.value_grid = np.copy(self.next_value_grid)

		if x0:
			self.x = x0
		if y0:
			self.y = y0
		if h0:
			self.h = h0
		if p_error:
			self.p_error = p_error
		if goal:
			self.goal = goal
		
		self.history = []
		self.add_history()
		print('MOVING')
		stopCondition = (self.x == self.goal[0]) and (self.y == self.goal[1]) and (self.h == self.goal[2] and match_h)
		while not stopCondition:
			mov = self.lookahead_grid[self.x, self.y, self.h]
			print('stepping')
			print (mov)
			mov, turn = mov[0], mov[1]
			self.move(mov, turn)
			self.add_history()
			stopCondition = (self.x == self.goal[0]) and (self.y == self.goal[1]) and (self.h == self.goal[2] and match_h) or (mov == 0)
			# self.plot_grid()

	def run_24b(self):
		self.value_iteration(0.9, 1, 4, 6, 0.0, (4,4), False)
		self.plot_grid()

	def timing_and_comparison(self):
		# policy iteration
		start = [time.time(), time.clock()]
		self.policy_iteration(0.9, 1, 4, 6, 0.0, (4,4), False)
		end = [time.time(), time.clock()]
		print("policy iteration time, .clock, .time: ")
		print (str(end[1] - start[1]) )
		print (str(end[0] - start[0]) )
		# save policy/value
		policyiter_policy = np.copy(self.lookahead_grid)
		policyiter_value = np.copy(self.value_grid)

		# value iteration
		start = [time.time(), time.clock()]
		self.value_iteration(0.9, 1, 4, 6, 0.0, (4,4), False)
		end = [time.time(), time.clock()]
		print("value iteration time, .clock, .time: ")
		print (str(end[1] - start[1]))
		print (str(end[0] - start[0]))
		# save policy/value
		valueiter_policy = np.copy(self.lookahead_grid)
		valueiter_value = np.copy(self.value_grid)

		# now compare policy
		if np.array_equal(valueiter_policy, policyiter_policy):
			print ("Policies from value and policy iteration are equal!")
		else:
			mask = np.not_equal(valueiter_policy, policyiter_policy)
			count_diff = np.sum(mask)
			print ("They differ by " + str(count_diff) + " elements")
	
	def run_25a(self):
		self.policy_iteration(0.9, 1, 4, 6, 0.1, (4,4), False)
		self.plot_grid()
	
	def run_25b(self):
		#errs = [0, 0.05,0.1,0.15,0.2, 0.25]
		errs = [0.15,0.2, 0.25]
		for err in errs:
			self.policy_iteration(0.9, 1, 4, 6, err, (4,4,6), True)
			print("Now at err = " + str(err))
			self.plot_grid()



