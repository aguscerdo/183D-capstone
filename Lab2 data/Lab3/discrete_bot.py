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

	
	def add_history(self):
		self.history.append([self.x, self.y, self.h])
	
	
	def update_h(self, h):
		if h < 0:
			h += 12
		self.h = h % 12
	
	
	def heading_to_direction(self):
		h = self.h
		if h in [11, 0, 1]:
			return 0
		elif h in [2, 3, 4]:
			return 1
		elif h in [5, 6, 7]:
			return 2
		else:
			return 3
		
	
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
	
	
	def state_difference(self, x_target, y_target, h_target):
		return x_target - self.x, y_target - self.y, h_target - self.h
	
	
	def move_probability(self, movement, turning, x_target, y_target, h_target):
		dx, dy, dh = self.state_difference(x_target, y_target, h_target)
		
		if np.abs(dx) + np.abs(dy) != 1:
			# Too far or in-place
			return 0
		
		h_dir = self.heading_to_direction()
		h_edge = (self.h + 1) % 3
		
		dx *= movement
		dy *= movement
		
		if dx == 1:
			if h_dir == 1:
				if h_edge == 1:
					return 1
				else:
					return 1 - self.p_error
			elif (h_dir == 0 and h_edge == 2) or (h_dir == 2 and h_edge == 0):
				return self.p_error
			else:
				return 0
		
		elif dx == -1:
			if h_dir == 3:
				if h_edge == 1:
					return 1
				else:
					return 1 - self.p_error
			elif (h_dir == 2 and h_edge == 2) or (h_dir == 0 and h_edge == 0):
				return self.p_error
			else:
				return 0
		
		elif dy == 1:
			if h_dir == 0:
				if h_edge == 1:
					return 1
				else:
					return 1 - self.p_error
			elif (h_dir == 1 and h_edge == 0) or (h_dir == 3 and h_edge == 2):
				return self.p_error
			else:
				return 0
		
		elif dy == -1:
			if h_dir == 2:
				if h_edge == 1:
					return 1
				else:
					return 1 - self.p_error
			elif (h_dir == 1 and h_edge == 2) or (h_dir == 3 and h_edge == 0):
				return self.p_error
			else:
				return 0
		return 0
		
	
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
		
		h = self.heading_to_direction()
		
		if h == 0:
			wall = self.update_y(movement*1)
		elif h == 1:
			wall = self.update_x(movement*1)
		elif h == 2:
			wall = self.update_y(movement*-1)
		else:
			wall = self.update_x(movement*-1)
		
		self.update_h(self.h + turning)
		
		
		