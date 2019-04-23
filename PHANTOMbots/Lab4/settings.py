robot_x = 0
robot_y = 0
angle = 0
obs = [[100, 100, 30, 30], [210, 300, 200, 80]]
idx = 0

def set_idx(i):
    global idx
    idx = i

def update_state(x,y,a):
    global robot_x, robot_y, angle

    robot_x = x
    robot_y = y
    angle = a


def get_state():
    global robot_x, robot_y, angle

    return [robot_x, robot_y, angle]


def experiment_obstacles():
    # 711, 482
    init = [60, 60, 0]
    global idx
    if idx == 0:
        # Box in the middle
        obs = [
            [320, 210, 70, 70]
        ]
        target = [500, 80]
    elif idx == 1:
        # parallel parking
        obs = [
            [200, 300, 50, 80],
            [420, 300, 60, 80]  # TODO fix x coord
        ]
        target = [315, 350, 0]
    elif idx == 2:
        obs = [
            [0, 215, 250, 50],
            [450, 215, 100, 50]
        ]
        target = [50, 350]
    elif idx == 3:
        obs = [
            [0, 200, 400, 50],
        ]
        target = [50, 350]
    elif idx == 4:
        obs = [
            [250, 0, 20, 100],
            [0, 250, 90, 20]
        ]
        target = [500, 80]
    elif idx == 5:
        obs = [
            [200, 0, 30, 250],
            [400, 150, 30, 332]
        ]
        target = [500, 80]
    else:
        return None

    if len(target) == 2:
        target.append(0)

    return init, obs, target