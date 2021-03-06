### ENVIRONMENT CLASS
#	size is [x,y], which gives the number of vertices in our grid 
#	vertices is the set of all vertices in our maze,
# 	a vertice is defined by its x and y coordinate ([x, y])
#	We assume that all adjacent vertices connected if they are in the grid
#	bots is a list of PhantomBots in our simulation

import matplotlib.pyplot as plt
import numpy as np
from phantomBot import PhantomBot
# TODO: change these later
presetSize = [10, 10]
presetVertices = []
for i in range(presetSize[0]):
	for j in range(presetSize[1]):
		if (i !=j and (i != 6 or j > 5) and (j!=9 or i <2) ) or ((j == 6 or j == 2) and i == 6):
			presetVertices.append([i,j])

class Environment:
	def __init__(self, printAll=False, size=None, vertices=None, bots=None ):
		self.printAll = printAll
		if (self.printAll):
			print("Print all mode on!")
		if size is None or vertices is None or bots is None:
			self.size = presetSize
			self.vertices = presetVertices
			pursuer1 = PhantomBot(printAll=False, pos=[0,1], pacman=False) 
			pursuer2 = PhantomBot(printAll=False, pos=[0,5]) 
			pursuer3 = PhantomBot(printAll=False, pos=[0,7]) 
			pursuer4 = PhantomBot(printAll=False, pos=[5,1]) 
			pacmanBot = PhantomBot(printAll=False, pos=[6,6], pacman=True, speed=1.2) 
			self.bots = [pursuer1, pursuer2, pursuer3, pursuer4, pacmanBot]
			if (self.printAll):
				print("Using preset size/edges/bots!")
		else:
			self.size = size
			self.vertices = vertices
			self.bots = bots
			if (self.printAll):
				print("Using grid of size " + str(self.size))

		self.verticeMatrix = np.zeros(self.size)
		self.set_vertice_matrix()
		self.occupiedVertices = []
		for bot in self.bots:
			self.occupiedVertices.append(bot.get_position())

	def set_vertice_matrix(self):
		"""
		verticeMatrix_i,j = 1 iff vertex at (i,j) in our maze
		"""
		self.verticeMatrix = np.zeros(self.size)
		for vertex in self.vertices:
			self.verticeMatrix[vertex[0], vertex[1]] = 1

	def load_grid(self, size, vertices):
		"""
		Loads the grid
		:param size: size of grid, tuple of (x, y)
		:param edges: list of edges, e in edges => e = (v1, v2) and vi = (xi, yi)
		:return:
		"""
		self.size = size
		self.vertices = vertices
		self.set_vertice_matrix()
	
		if (self.printAll):
			print("Loaded new grid, size is " + str(self.size))

	def plot_grid(self):
		"""
		Plots the edges by plotting a line for each edge
		"""
		ax = plt.gca()
		if (self.printAll):
			print("Plotting Grid!")
		for i in range(self.size[0]):
			for j in range(self.size[1]):
				#plot vertex i,j
				if (self.verticeMatrix[i,j]):
					ax.plot(i, j, 'bx', label='point')
				#plot line from i,j -> i+1,j 
				if (i+1 < self.size[0] and self.verticeMatrix[i,j] and self.verticeMatrix[i+1,j]):
					xs = [i, i+1]
					ys = [j, j]
					ax.plot(xs, ys, 'r')
				#plot line from i,j -> i,j+1
				if (j+1 < self.size[1] and self.verticeMatrix[i,j] and self.verticeMatrix[i,j+1]):
					xs = [i, i]
					ys = [j, j+1]
					ax.plot(xs, ys, 'r')
		#plot bots:
		radius = 0.1
		for bot in self.bots:
			if bot.is_pacman():
				circle = plt.Circle(bot.get_position(), radius, color='yellow')
			else:
				circle = plt.Circle(bot.get_position(), radius, color='blue')
			ax.add_artist(circle)

		plt.title('The Maze')
		# x from -0.5 -> size_x - 0.5, y from -0.5 -> size_y - 0.5
		# -0.5 to show full grid
		plt.axis([-0.5, self.size[0]-0.5, -0.5, self.size[1]-0.5])
		plt.show()

	def legal_move(self, bot, end_pos):
		"""
		Checks if move if legal
		:param bot: which bot is moving, i
		:param end_pos: where a bot will end, (x1, y1)
		:return: True if move is legal else false
		"""
		start_pos = self.bots[bot].get_position()
		# True if dist == 1 and both vertices in set and no collision
		valid_move = True
		valid_move &= (self.dist(start_pos, end_pos) == 1)
		valid_move &= (self.verticeMatrix[end_pos[0], end_pos[1]] == 1)
		valid_move &= (self.verticeMatrix[start_pos[0], start_pos[1]] == 1)
		# check collisions, TODO maybe change this(?)
		for i in range(len(self.bots)):
			valid_move &= (i==bot or dist(start_pos, self.bots[i].get_position()) > 0)
		if (self.printAll):
			print("Moving from " + str(start_pos) + " to " + str(end_pos) + " is: ")
			if (valid_move):
				print("legal")
			else:
				print("illegal")
		return valid_move
	
	def move(self, bot, end_pos):
		"""
		Moves bot to location
		:param bot: which bot is moving, i
		:param end_pos: where a bot will end, (x1, y1)
		"""
		self.bots[bot].move(end_pos)
		self.occupiedVertices[bot] = end_pos

	def dist(self, a, b):
		"""
		Gets railroad/manhatten/L1 distance
		:param a: (x0, y0)
		:param b: (x1, y1)
		"""
		return np.abs(a[0] - b[0]) + np.abs(a[1] - b[1])

env = Environment()
env.plot_grid()

