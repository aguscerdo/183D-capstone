from paperbot import PaperBot
import numpy as np


def main():
	n = 5
	n2 = 5
	n3 = 100
	ul = [180] * n #+ [180] * n2 + [60] * n3
	ur = [100] * n# + [0] * n2   + [0] * n3
	
	p = PaperBot(100, 0, 0)
	
	for i in range(min(len(ul), len(ur))):
		p.move(ul[i], ur[i])

	p.plot_history()
	
	

if __name__ == '__main__':
	main()