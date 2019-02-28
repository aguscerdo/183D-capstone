from L4Bot import L4Bot
import threading
import time
robot_x = 0
robot_y = 0
angle = 0

class Thread_A(threading.Thread):
	def __init__(self, name):
		threading.Thread.__init__(self)
		self.name = name

	def run(self):
		global robot_x, robot_y, angle 
		robot_x = 1
		time.sleep(1)
		robot_x = 3
		robot_y = 1
		angle = 1


		
class Thread_B(threading.Thread):
	def __init__(self, name):
		threading.Thread.__init__(self)
		self.name = name

	def run(self):
		global robot_x, robot_y, angle
		print(robot_x)
		time.sleep(1)
		print(robot_x)
		time.sleep(1)
		print(robot_x)
		db = L4Bot(711, 482, 600.0, 300.0, 0.0)  #Test Area is 711mm by 482mm
		obs = [[100, 100, 30, 30], [210, 300, 200, 80]]
		db.load_obstacles(obs)
		#db.plot()
		start = [600,300,0]
		goal = [100,400,0]
		db.run(start, goal, branches_per_evolution=10,num_evolutions=1)
		

a = Thread_A("camera_position_tracking")
b = Thread_B("RRT_run")

b.start()
a.start()

a.join()
b.join()
