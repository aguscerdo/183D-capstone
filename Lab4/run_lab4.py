from L4Bot import L4Bot

def main():
	db = L4Bot(750, 500, 600.0, 300.0, 0.0)
	obs = [[100, 100, 30, 30], [210, 300, 200, 80]]
	db.load_obstacles(obs)
	#db.plot()
	db.RRT(num_branches=20)
	db.visualise_RRT()


if __name__ == '__main__':
	main()