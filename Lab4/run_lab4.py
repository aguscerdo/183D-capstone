from L4Bot import L4Bot

def main():
	db = L4Bot(711, 482, 600.0, 300.0, 0.0)  #Test Area is 711mm by 482mm
	obs = [[100, 100, 30, 30], [210, 300, 200, 80]]
	db.load_obstacles(obs)
	#db.plot()
	db.RRT(num_branches=1000)
	db.visualise_RRT()
	p = db.findPath()
	#print(p)

	commands = [[120, 120], [70, 70]]

	t = time.time()
	delta = 2.

	i = 0

	while 1:
		t0 = time.time()

		if t0 > t+delta:
			i += 1
			i %= 2
			c = commands[i]
			db.send_socket(c[0], c[1])
			t = t0


if __name__ == '__main__':
	main()
