from Lab4.L4Bot import L4Bot


def main():
	db = L4Bot(750, 500, 300, 300, 0)
	obs = [[100, 100, 30, 30], [210, 300, 200, 80]]
	db.load_obstacles(obs)
	db.plot()


if __name__ == '__main__':
	main()