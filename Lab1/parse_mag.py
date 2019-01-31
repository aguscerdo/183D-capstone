import matplotlib.pyplot as plt
import numpy as np 
import re
import sys

def main_parser(use_mag, s):
	with open(filename, 'r') as fp:
		lines = fp.read()

	lines = lines.split()
	N = len(lines)

	readings = []

	# Line format: 
	# (4:31:41 PM) : RX = Mx,My: (-1, 72). Deg: (162.000000)
	# (4:27:00 PM) : RX = L1 (138); L2(276)
	for i in range(0, N):	# [^:]\d+\.?\d*[^:]
		if use_mag:
			if re.search('Deg:')
				content = re.find(r'Deg: \((\d+\.?\d*)\)', lines[i])
				readings.append(content)
		else:
			if s == 1:
				if re.search('L1 ')
					content = re.find(r'L1 \((\d+\.?\d*)\)', lines[i])
					readings.append(content)
				else:
					content = re.find(r'L2\((\d+\.?\d*)\)', lines[i])
					readings.append(content)

	mean = np.mean(readings)
	var = np.variance(readings)

	print('Mean: {} ~~ Variance: {}'.format(mean, var))

if __name__ == '__main__':
	if len(sys.argv) == 1:
		print('(function) FILE (mag|lid) (1|2) for lidars')
	fn = sys.argv[1]
	if len(sys.argv) < 3:
		mag = 1
	else:
		if sys.argv[2] == 'mag':
			mag = 1
		else:
			mag = 0
			sensor = sys.argv[3]	# 1 or 2


	main_parser(mag, sensor)