import numpy as np 
import re
import sys

def main_parser(filename, use_mag, s):
	print(filename)
	with open(filename, 'r') as f:
		lines = f.readlines()

	N = len(lines)
	readings = []

	# Line format: 
	# (4:31:41 PM) : RX = Mx,My: (-1, 72). Deg: (162.000000)
	# (4:27:00 PM) : RX = L1 (138); L2(276)
	for i in range(0, N):	# [^:]\d+\.?\d*[^:]

		if use_mag:
			if re.search('Deg:', lines[i]):
				content = re.search(r'Deg: \((\d+\.?\d*)\)', lines[i]).group(1)
				readings.append(float(content))
		else:
			if re.search('L1 ', lines[i]):
				if s == 1:
					content = re.search(r'L1 \((\d+)\)', lines[i]).group(1)
					readings.append(float(content))
				else:
					content = re.search(r'L2\((\d+)\)', lines[i]).group(1)
					readings.append(float(content))

	mean = np.mean(readings)
	var = np.var(readings)
	print('Mean: {} ~~ Variance: {}'.format(mean, var))

if __name__ == '__main__':
	sensor = None
	if len(sys.argv) == 1:
		print('(function) FILE (mag|lid) (1|2) for lidars')
	fn = sys.argv[1]
	if len(sys.argv) < 3:
		mag = True
	else:
		if sys.argv[2] == 'mag':
			mag = True
		else:
			mag = False
			sensor = sys.argv[3]	# 1 or 2


	main_parser(fn, mag, sensor)