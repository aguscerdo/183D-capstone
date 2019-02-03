import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np 
import re






def main_parser(filename):
	with open(filename, 'r') as fp:
		lines = fp.read()

	lines = lines.split('\n')
	N = len(lines)
	
	Z_est = []
	X_est = []
	
	Mag = []
	Lidars = []

	# Line format:
	# Predicted (X, Y, Th) Measured (X, Y, Th)
	for i in range(0, N):	# [^:]\d+\.?\d*[^:]
		l = lines[i]
		if re.search(r'Kalman', l):
			# X_est
			findings = re.search(r'\-\>\(([\d,\.-]+), zest', l).group(1)
			ss = findings.split(',')
			X_est.append(ss)
			
			# Z_est
			findings = re.search(r'Lf,M\)=\(([\d,\.-]+)\)', l).group(1)
			ss = findings.split(',')
			Z_est.append(ss)
			
		elif re.search(r'Deg:', l):
			findings = re.search(r'Deg: \(([\d\.-]+)\)', l).group(1)
			Mag.append(findings)
		elif re.search(r'RX = L1', l):
			findings = re.search(r'L1 \(([\d\.]+)\); L2\(([\d\.]+)\)', l)
			L1 = findings.group(1)
			L2 = findings.group(2)
			Lidars.append([L1, L2])
		else:
			continue
		
	N = np.min([len(X_est), len(Z_est), len(Mag), len(Lidars)])
	base_name = filename.split('.')[0]
	
	
	X_est = np.asarray(X_est, float)[:N]
	Z_est = np.asarray(Z_est, float)[:N]
	Mag = np.asarray(Mag, float)[:N]
	Lidars = np.asarray(Lidars, float)[:N]

	
	r = np.arange(N)
	# Measurements vs Predicted
	# 1. Magnetometer
	plt.plot(r, Mag, '-o', c='r', label='Reading')
	plt.plot(r, Z_est[:, 2], '-o', c='b', label='Predicted')
	plt.legend()
	plt.title('Predicted and Measured Bearing')
	plt.xlabel('Step (units)')
	plt.ylabel('Bearing (deg)')
	plt.savefig('ktest/{}_Mag.png'.format(base_name))
	plt.show()

	# 2. Lidar Front
	plt.plot(r, Lidars[:, 1], '-o', c='r', label='Reading')
	plt.plot(r, Z_est[:, 1], '-o', c='b', label='Predicted')
	plt.legend()
	plt.title('Predicted and Measured Distance: Front')
	plt.xlabel('Step (units)')
	plt.ylabel('Distance (mm)')
	plt.savefig('ktest/{}_dist_LF.png'.format(base_name))
	plt.show()

	# 2. Lidar Right
	plt.plot(r, Lidars[:, 0], '-o', c='r', label='Reading')
	plt.plot(r, Z_est[:, 0], '-o', c='b', label='Predicted')
	plt.legend()
	plt.title('Predicted and Measured Distance: Right')
	plt.xlabel('Step (units)')
	plt.ylabel('Distance (mm)')
	plt.savefig('ktest/{}_dist_LR.png'.format(base_name))
	plt.show()
	
	
	# Position
	X = X_est[:, 0]
	Y = X_est[:, 1]
	Th = X_est[:, 2]
	
	plt.scatter(X, Y, c=r, cmap=cm.get_cmap('YlGn'))
	
	plt.title('Position over time')
	plt.xlabel('X (mm)')
	plt.ylabel('Y (mm)')
	plt.savefig('ktest/{}_position.png'.format(base_name))
	plt.show()
	


if __name__ == '__main__':
	fn = 'ktest_3.txt'
	main_parser(fn)