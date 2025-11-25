#!/usr/bin/env python3
import rospy
import math
import heapq
from geometry_msgs.msg import Pose2D, Point

# same static obstacles as nav_sim
OBSTACLES = [
    (1.2, 0.5, 0.4),
    (2.0, 1.5, 0.4),
    (2.5, 0.5, 0.4),
]

# grid parameters (world coords)
X_MIN, X_MAX = -1.0, 4.0
Y_MIN, Y_MAX = -1.0, 4.0
RES = 0.2  # cell size

def world_to_cell(x, y):
    gx = int((x - X_MIN) / RES)
    gy = int((y - Y_MIN) / RES)
    return gx, gy

def cell_to_world(gx, gy):
    x = X_MIN + gx * RES
    y = Y_MIN + gy * RES
    return x, y

def build_grid():
    nx = int((X_MAX - X_MIN) / RES)
    ny = int((Y_MAX - Y_MIN) / RES)
    grid = [[0 for _ in range(ny)] for _ in range(nx)]

    # mark obstacle cells
    for (ox, oy, r) in OBSTACLES:
        for gx in range(nx):
            for gy in range(ny):
                wx, wy = cell_to_world(gx, gy)
                d = math.sqrt((wx - ox)**2 + (wy - oy)**2)
                if d < r + 0.1:
                    grid[gx][gy] = 1
    return grid

def astar(grid, start, goal):
    nx, ny = len(grid), len(grid[0])

    def in_bounds(p):
        x, y = p
        return 0 <= x < nx and 0 <= y < ny

    def is_free(p):
        x, y = p
        return grid[x][y] == 0

    def neighbors(p):
        x, y = p
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
            q = (x+dx, y+dy)
            if in_bounds(q) and is_free(q):
                yield q

    def h(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_set = []
    heapq.heappush(open_set, (0 + h(start, goal), 0, start))
    came = {start: None}
    gscore = {start: 0}

    while open_set:
        _, cost, cur = heapq.heappop(open_set)
        if cur == goal:
            path = []
            p = cur
            while p is not None:
                path.append(p)
                p = came[p]
            return list(reversed(path))

        for nb in neighbors(cur):
            t = gscore[cur] + 1
            if nb not in gscore or t < gscore[nb]:
                gscore[nb] = t
                heapq.heappush(open_set, (t + h(nb, goal), t, nb))
                came[nb] = cur
    return []

class PathPlanner:
    def __init__(self):
        self.grid = build_grid()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal_x = 3.0
        self.goal_y = 2.0

        self.path_cells = []
        self.path_index = 0

        self.pose_sub = rospy.Subscriber("/robot_pose", Pose2D, self.pose_cb)
        self.goal_sub = rospy.Subscriber("delivery_goal", Point, self.goal_cb)
        self.next_goal_pub = rospy.Publisher("/next_goal", Point, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)  # 10 Hz

    def pose_cb(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y

    def goal_cb(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.compute_path()

    def compute_path(self):
        sx, sy = world_to_cell(self.robot_x, self.robot_y)
        gx, gy = world_to_cell(self.goal_x, self.goal_y)
        path = astar(self.grid, (sx, sy), (gx, gy))
        self.path_cells = path
        self.path_index = 0
        rospy.loginfo("A* path len: %d", len(path))

    def update(self, event):
        if not self.path_cells:
            return

        # current waypoint in world coords
        cx, cy = cell_to_world(*self.path_cells[self.path_index])

        # check distance to waypoint
        dx = cx - self.robot_x
        dy = cy - self.robot_y
        d = math.sqrt(dx*dx + dy*dy)

        # move to next waypoint if close enough
        if d < 0.15 and self.path_index < len(self.path_cells) - 1:
            self.path_index += 1
            cx, cy = cell_to_world(*self.path_cells[self.path_index])

        p = Point()
        p.x = cx
        p.y = cy
        p.z = 0.0
        self.next_goal_pub.publish(p)

if __name__ == "__main__":
    rospy.init_node("path_planner")
    planner = PathPlanner()
    rospy.spin()
