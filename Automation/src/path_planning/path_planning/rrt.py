import numpy as np
import math
import random
import rclpy

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, map_limits, obstacle_list,
                 step_len=0.5, goal_sample_rate=5, max_iter=500,
                 search_radius=2.0):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.min_rand = map_limits[0]
        self.max_rand = map_limits[1]
        self.obstacle_list = obstacle_list  # list of obstacles [x,y,w,h]
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate  # percentage
        self.max_iter = max_iter
        self.search_radius = search_radius
        self.node_list = [self.start]

    def planning(self):
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node)

            if self.check_collision(new_node):
                neighbors = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, neighbors)
                self.node_list.append(new_node)
                self.rewire(new_node, neighbors)

                if self.calc_dist_to_goal(new_node.x, new_node.y) <= self.step_len:
                    final_node = self.steer(new_node, self.goal)
                    if self.check_collision(final_node):
                        return self.generate_final_course(len(self.node_list) - 1)
        return None  # no path found

    def steer(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        if distance > self.step_len:
            theta = math.atan2(dy, dx)
            new_x = from_node.x + self.step_len * math.cos(theta)
            new_y = from_node.y + self.step_len * math.sin(theta)
        else:
            new_x = to_node.x
            new_y = to_node.y

        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        new_node.cost = from_node.cost + self.calc_distance(from_node, new_node)
        return new_node

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(random.uniform(self.min_rand, self.max_rand),
                       random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = Node(self.goal.x, self.goal.y)
        return rnd

    def get_nearest_node_index(self, node):
        dlist = [self.calc_distance(node, n) for n in self.node_list]
        min_index = dlist.index(min(dlist))
        return min_index

    def calc_distance(self, n1, n2):
        dx = n1.x - n2.x
        dy = n1.y - n2.y
        return math.hypot(dx, dy)

    def check_collision(self, node):
        if node is None:
            return False
        for (ox, oy, w, h) in self.obstacle_list:
            if ox <= node.x <= ox + w and oy <= node.y <= oy + h:
                return False  # collision
        return True

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.search_radius * math.sqrt((math.log(nnode) / nnode))
        dlist = [self.calc_distance(new_node, node) for node in self.node_list]
        neighbors = [i for i, d in enumerate(dlist) if d <= r]
        return neighbors

    def choose_parent(self, new_node, neighbors):
        if not neighbors:
            return new_node
        costs = []
        for i in neighbors:
            neighbor = self.node_list[i]
            if self.check_path_collision(neighbor, new_node):
                costs.append(neighbor.cost + self.calc_distance(neighbor, new_node))
            else:
                costs.append(float('inf'))
        min_cost = min(costs)
        min_ind = neighbors[costs.index(min_cost)]
        if min_cost == float('inf'):
            return new_node
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node

    def check_path_collision(self, from_node, to_node):
        # Check collision between two nodes by discretizing the path
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        steps = int(math.floor(distance / 0.1))
        if steps == 0:
            return self.check_collision(to_node)
        for i in range(steps):
            x = from_node.x + dx * i / steps
            y = from_node.y + dy * i / steps
            point = Node(x, y)
            if not self.check_collision(point):
                return False
        return True

    def rewire(self, new_node, neighbors):
        for i in neighbors:
            neighbor = self.node_list[i]
            if neighbor == new_node.parent:
                continue
            edge_cost = self.calc_distance(new_node, neighbor)
            new_cost = new_node.cost + edge_cost
            if neighbor.cost > new_cost and self.check_path_collision(new_node, neighbor):
                neighbor.parent = new_node
                neighbor.cost = new_cost

    def calc_dist_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        return math.hypot(dx, dy)

    def generate_final_course(self, goal_index):
        path = []
        node = self.node_list[goal_index]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        return path[::-1]

def main(args=None):
    rclpy.init(args=args)

    # Example usage or ROS node initialization here:
    start = (1, 1)
    goal = (8, 8)
    map_limits = (0, 10)
    obstacles = [(3, 3, 1, 1), (5, 5, 1, 2), (7, 2, 1, 3)]

    rrt_star = RRTStar(start, goal, map_limits, obstacles)
    path = rrt_star.planning()

    if path:
        print("Found path:")
        for p in path:
            print(p)
    else:
        print("No path found")

    rclpy.shutdown()



if __name__ == '__main__':
    main()