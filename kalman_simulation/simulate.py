from paperbot import PaperBot
import numpy as np


def main():
	n = 15
	n2 = 5
	n3 = 5
	ul = [180] * n + [0] * n2 + [0] * n3 + [100] * n3 + [0] * 2
	ur = [100] * n + [0] * n2 + [180] * n3 + [100] * n3 + [180] * 2
	
	p = PaperBot(100, 0, 0)
	
	for i in range(min(len(ul), len(ur))):
		p.move(ul[i], ur[i])
		
		# if i % 5 == 0:
		# 	p.plot_history()

	p.plot_history()
	
	

if __name__ == '__main__':
	main()