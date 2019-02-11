from paperbot import PaperBot
# TODO fix dependency if needed with below
# import sys
# sys.path.append('./kalman_simulation')
import numpy as np


def main():
	"""
	Performs a simulation session
	:return:
	"""
	
	# Control different movements repeated n times
	n1 = 30
	v1L = 180
	v1R = 95
	
	n2 = 5
	v2L = 0
	v2R = 0
	
	n3 = 20
	v3L = 0
	v3R = 90
	
	n4 = 20
	v4L = 101
	v4R = 99
	
	ul = [v1L] * n1 + [v2L] * n2 + [v3L] * n3 + [v4L] * n4
	ur = [v1R] * n1 + [v2R] * n2 + [v3R] * n3 + [v4R] * n4
	
	# Initialize paperbot at X, Y, Th
	p = PaperBot(100, 100, 0)
	
	for i in range(min(len(ul), len(ur))):
		p.move(ul[i], ur[i])
		
		# Plot every 5
		if i % 5 == 0:
			p.plot_history()
		# p.plot_history()

	# Final plotting
	p.plot_history()
	
	

if __name__ == '__main__':
	main()