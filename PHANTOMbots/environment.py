### ENVIRONMENT CLASS
#	size is [x,y], which gives the number of vertices in our grid 
#	vertices is the set of all vertices in our maze,
# 	a vertice is defined by its x and y coordinate ([x, y])
#	We assume that all adjacent vertices connected if they are in the grid

import matplotlib.pyplot as plt
import numpy as np
# TODO: change these later
presetSize = [3, 3]
v1 = [0,0]
v2 = [0,1]
v3 = [1,0]
v4 = [1,1]
v5 = [2,0]
v6 = [2,1]
v7 = [2,2]
presetVertices = [v1, v2, v4, v5, v6, v7]

class Environment:
	def __init__(self, printAll=False, size=None, vertices=None ):
		self.printAll = printAll
		if (self.printAll):
			print("Print all mode on!")
		if size is None or vertices is None:
			self.size = presetSize
			self.vertices = presetVertices
			if (self.printAll):
				print("Using preset size/edges!")
		else:
			self.size = size
			self.vertices = vertices
			if (self.printAll):
				print("Using grid of size " + str(self.size))

		self.verticeMatrix = np.zeros(self.size)
		self.set_vertice_matrix()

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
		if (self.printAll):
			print("Plotting Grid!")
		for i in range(self.size[0]):
			for j in range(self.size[1]):
				#plot vertex i,j
				if (self.verticeMatrix[i,j]):
					plt.plot(i, j, 'bx', label='point')
				#plot line from i,j -> i+1,j 
				if (i+1 < self.size[0] and self.verticeMatrix[i,j] and self.verticeMatrix[i+1,j]):
					xs = [i, i+1]
					ys = [j, j]
					plt.plot(xs, ys, 'r')
				#plot line from i,j -> i,j+1
				if (j+1 < self.size[1] and self.verticeMatrix[i,j] and self.verticeMatrix[i,j+1]):
					xs = [i, i]
					ys = [j, j+1]
					plt.plot(xs, ys, 'r')

		plt.title('The Maze')
		# x from -0.5 -> size_x - 0.5, y from -0.5 -> size_y - 0.5
		# -0.5 to show full grid
		plt.axis([-0.5, self.size[0]-0.5, -0.5, self.size[1]-0.5])
		plt.show()

	def legal_move(self, start_pos, end_pos):
		"""
		Checks if move if legal
		:param start_pos: where a bot starts, (x0, y0)
		:param end_pos: where a bot will end, (x1, y1)
		:return: True if move is legal else false
		"""
		# True if dist == 1 and both vertices in set
		valid_move = True
		valid_move &= (self.dist(start_pos, end_pos) == 1)
		valid_move &= (self.verticeMatrix[end_pos[0], end_pos[1]] == 1)
		valid_move &= (self.verticeMatrix[start_pos[0], start_pos[1]] == 1)
		if (self.printAll):
			print("Moving from " + str(start_pos) + " to " + str(end_pos) + " is: ")
			if (valid_move):
				print("legal")
			else:
				print("illegal")
		return valid_move
	
	def dist(self, a, b):
		"""
		Gets railroad/manhatten/L1 distance
		:param a: (x0, y0)
		:param b: (x1, y1)
		"""
		return np.abs(a[0] - b[0]) + np.abs(a[1] - b[1])


env = Environment(printAll=True)
env.plot_grid()
env.legal_move(v1, v2)
env.legal_move(v1, v3)
env.legal_move(v1, v4)
