import numpy as np
import matplotlib.pyplot as plt
from numpy import np

class DiscreteBot:
	def __init__(self, W, L, x0, y0, h0, p_error=0.05):
		# Initialize states
		self.L = L
		self.W = W
		
		self.x = x0
		self.y = y0
		self.h = h0
		self.history = [[x0, y0, h0]]
		
		self.p_error=p_error
		
		self.S = np.zeros((L, W))   # N_s = W * L
		self.build_grid()
	
	
	def build_grid(self):
		for i in range(self.L):
			self.S[i, 0] = -100
			self.S[i, self.W-1] = -100
		for i in range(self.W):
			self.S[0, 1] = -100
			self.S[self.L-1, 0] = -100
		
		self.S[3, 3] = self.S[3, 4] = -1
		self.S[4, 4] = 1
		
	
	def add_history(self):
		self.history.append([self.x, self.y, self.h])
	
	
	def update_h(self, h):
		if h < 0:
			h += 12
		self.h = h % 12
	
	@staticmethod
	def heading_to_direction(h_s):
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
			self.x += diff
			return True
		else:
			return False
	
	
	def update_y(self, diff):
		if 0 <= self.y + diff < self.W:
			self.y += diff
			return True
		else:
			return False
	
	@staticmethod
	def state_difference(x_s, y_s, h_s, x_target, y_target, h_target):
		return x_target - x_s, y_target - y_s, h_target - h_s
	
	
	def move_probability(self, x_s, y_s, h_s, movement, turning, x_target, y_target, h_target):
		"""
		Solution 2.1.c: Receives state and input and returns probability
		
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
		if movement == 0:
			return None
		elif not (0<= x_s < self.L and 0<= y_s < self.W):
			return None
		
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
					
					prob[di+1, dj+1, dh+2] = self.move_probability(
							movement, turning, x_s+di, y_s+dj, h_s+dh)
					
		next_state = np.argmax(prob)
		return x_s + next_state[0] - 1, y_s + next_state[1] - 1, h_s + next_state[2] - 2
	
	
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
		# Heading is straight up
		if mode == 0:
			if turning == 0:
				if dh_abs == 1:
					ret = self.p_error
				else:
					ret = 1 - 2*self.p_error
			elif turning == -1:
				if dh == -2 or dh == 0:
					ret = self.p_error
				elif dh == -1:
					ret = 1 - 2 * self.p_error

			else:
				if dh == 2 or dh == 0:
					ret = self.p_error
				elif dh == 1:
					ret = 1 - 2 * self.p_error
					
		# Heading is an edge <-1  +1> but went to forward block
		elif mode == 1:
			if turning == 0:    # No turning
				if heading_edge == 0:   # Left edge
					if dh == 0:
						ret = 1 - 2*self.p_error
					elif dh == 1:
						ret = self.p_error

						
				elif heading_edge == 2:
					if dh == 0:
						ret = 1 - 2*self.p_error
					elif dh == -1:
						ret = self.p_error

			elif turning == 1:
				if heading_edge == 0:
					if dh == 1:
						ret = 1 - 2*self.p_error
					elif dh == 2:
						ret = self.p_error

				if heading_edge == 2:
					if dh == 0:
						ret = 1 - 2*self.p_error
					elif dh == -1:
						ret = self.p_error

			elif turning == -1:
				if heading_edge == 0:
					if dh == -1:
						ret = 1 - 2*self.p_error
					elif dh == -2:
						ret = self.p_error
						
		elif mode == 2: # On an edge of another heading
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
		
	
	def reward(self, x_s, y_s):
		"""
		2.2.a
		:param x_s: state x position
		:param y_s: state y position
		:return:
		"""
		return self.S[x_s, y_s]
	
	
	def move(self, movement, turning):
		"""
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
			wall = self.update_y(movement*1)
		elif h == 1:
			wall = self.update_x(movement*1)
		elif h == 2:
			wall = self.update_y(movement*-1)
		else:
			wall = self.update_x(movement*-1)
		
		self.update_h(self.h + turning)
		
		
		