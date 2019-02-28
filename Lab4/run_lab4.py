from L4Bot import L4Bot

def main():
	db = L4Bot(711, 482, 600.0, 300.0, 0.0)  #Test Area is 711mm by 482mm
	obs = [[100, 100, 30, 30], [210, 300, 200, 80]]
	db.load_obstacles(obs)
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

if __name__ == '__main__':
	main()
