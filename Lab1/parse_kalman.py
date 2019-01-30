import matplotlib.pyplot as plt
import numpy as np 
import re






def main_parser():
	filename = 'kalman_dump.txt'

	with open(filename, 'r') as fp:
		lines = fp.read()

	lines = lines.split()
	N = len(lines)
	X = np.zeros((N, 2))
	Y = np.zeros((N, 2))
	Th = np.zeros((N, 2))

	# Line format:
	# Predicted (X, Y, Th) Measured (X, Y, Th)
	for i in range(0, N):	# [^:]\d+\.?\d*[^:]
		content = re.findall(r'\d+\.?\d*', lines[i])
		if len(content) != 6:
			continue

		X[i][0] = content[0]
		X[i][1]	= content[3]

		Y[i][0] = content[1]
		Y[i][1]	= content[4]

		Th[i][0] = content[2]
		Th[i][1] = content[5]

	r = np.arange(N)
	plt.scatter(r, X[:, 0], c='r')
	plt.scatter(r, X[:, 1], c='b')
	plt.title('Predicted vs Measured X')
	plt.legend()
	plt.savefig('kalman_X.png')

	plt.cla()

	plt.scatter(r, Y[:, 0], c='r')
	plt.scatter(r, Y[:, 1], c='b')
	plt.title('Predicted vs Measured Y')
	plt.legend()
	plt.savefig('kalman_Y.png')

	plt.cla()

	plt.scatter(r, Th[:, 0], c='r')
	plt.scatter(r, Th[:, 1], c='b')
	plt.title('Predicted vs Measured Theta')
	plt.legend()
	plt.savefig('kalman_Th.png')

	plt.cla()

	plt.scatter(X[:, 0], Y[:, 0], c='r')
	plt.scatter(X[:, 1], Y[:, 1], c='b')
	plt.title('Prediction vs Measurements on Box')
	plt.legend()

	plt.savefig('kalman_Box.png')


if __name__ == '__main__':
	main_parser()