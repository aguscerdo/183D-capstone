from L4Bot import L4Bot
import time

def main():
	db = L4Bot(711, 482, 50.0, 50.0, 0.0)  #Test Area is 711mm by 482mm
	obs = [[100, 100, 30, 30], [210, 300, 200, 80]]

	start, obs, goal = db.experiment_obstacles(5)
	db.load_obstacles(obs)
	# db.set_pos(goal[0], goal[1])
	#db.plot()
	start = [600,300,0]
	goal = [100,400,0]
	db.reverse_RRT(goal_state=goal, num_branches=1000)
	db.visualise_RRT()
	#p = db.findPath(start=start, goal=goal)
	#db.RRT(start_state=start, num_branches=1000)
	#p = db.findPath(start=start, goal=goal)
	#db.visualise_RRT()
	#p = db.findPath()
	#db.run([600,300,0],[600,100,0],branches_per_evolution=40,num_evolutions=1)
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
