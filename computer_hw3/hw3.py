import time
import operator
from typing import NamedTuple
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import math


class Config(NamedTuple):
    x: float
    y: float


class MapEnvironment:

    def __init__(self, mapfile):

        # Obtain the boundary limits
        self.map = np.loadtxt(mapfile)
        self.xlimit = [0, np.shape(self.map)[1] - 1]
        self.ylimit = [0, np.shape(self.map)[0] - 1]

        self._show_map()

    def _show_map(self):
        plt.imshow(self.map, interpolation='nearest')
        plt.xlabel('x')
        plt.ylabel('y')

    def compute_distance(self, start_config, end_config):
        return math.sqrt((start_config.x - end_config.x)**2 + (start_config.y - end_config.y)**2)
        # return np.sqrt(np.square(start_config.x - end_config.x) + np.square(start_config.y - end_config.y))
        # The following is (surprisingly?) slower
        # return np.sqrt(np.sum(np.square(np.array(start_config) - np.array(end_config))))

    def check_state_validity(self, config):
        # check within limit
        if config.x < self.xlimit[0] or config.x > self.xlimit[1] or \
                config.y < self.ylimit[0] or config.y > self.ylimit[1]:
            return False
        # check collision
        # x, y = int(np.round(config.x)), int(np.round(config.y))
        x, y = int(round(config.x)), int(round(config.y))
        return not self.map[y][x]

    def check_edge_validity(self, config1, config2, visualize=False, pts=50):
        if visualize:
            self._show_map()
            plt.plot(config1.x, config1.y, 'o', color='w')
            plt.plot(config2.x, config2.y, 'o', color='w')
            plt.plot([config1.x, config2.x], [config1.y, config2.y], color='w')

        # Check validity of pts equally spaced points along the edge
        # arr = np.linspace(0, 1, pts)
        # np.random.shuffle(arr)
        # for alpha in arr:
        for _ in range(100 * int(np.ceil(self.compute_distance(config1, config2)))):
            alpha = np.random.uniform()
            new_x = alpha * config1.x + (1.0 - alpha) * config2.x
            new_y = alpha * config1.y + (1.0 - alpha) * config2.y
            if not self.check_state_validity(Config(new_x, new_y)):
                return False

        return True

    def visualize_plan(self, plan):
        self._show_map()
        plt.plot(plan[0].x, plan[0].y, 'o', color='g')
        plt.plot(plan[-1].x, plan[-1].y, 'o', color='r')
        for first_config, second_config in zip(plan, plan[1:]):
            plt.plot([first_config.x, second_config.x], [first_config.y, second_config.y], color='w')
        plt.show()


class RRTTree:

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.vertices = []
        self.edges = dict()

    def get_root_id(self):
        return 0

    def get_nearest_vertex(self, config):
        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.compute_distance(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]

    def add_vertex(self, config):
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def add_edge(self, sid, eid):
        self.edges[eid] = sid


class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

    def sample_random_config(self, goal_pose=None, r=1.0):
        if goal_pose is None:
            low_x, high_x = self.planning_env.xlimit
            low_y, high_y = self.planning_env.ylimit
        else:
            low_x = goal_pose.x - r
            high_x = goal_pose.x + r
            low_y = goal_pose.y - r
            high_y = goal_pose.y + r

        x = np.random.uniform(low=low_x, high=high_x)
        y = np.random.uniform(low=low_y, high=high_y)
        return Config(x, y)

    def extend_and_check(self, home, target, eta):
        x_new = (1.0 - eta) * home.x + eta * target.x
        y_new = (1.0 - eta) * home.y + eta * target.y
        if self.planning_env.check_edge_validity(home, Config(x_new, y_new)):
            return Config(x_new, y_new)
        else:
            return None

    def post_process(self, plan, pp_steps=20):
        for _ in range(pp_steps):
            if len(plan) < 3:
                return plan
            vid = np.random.choice(np.arange(1, len(plan) - 1))
            v = plan[vid]
            v_prev = plan[vid - 1]
            v_next = plan[vid + 1]
            last_points = []
            edges = True
            for ind, eta in enumerate(np.linspace(1, 0, 10)):
                p_prev = Config((1.0 - eta) * v.x + eta * v_prev.x, (1.0 - eta) * v.y + eta * v_prev.y)
                p_next = Config((1.0 - eta) * v.x + eta * v_next.x, (1.0 - eta) * v.y + eta * v_next.y)
                segment_length = self.planning_env.compute_distance(p_prev, p_next)
                found = self.planning_env.check_edge_validity(p_prev, p_next, pts=100 * int(np.ceil(segment_length)))
                if found:
                    last_points = [p_prev, p_next]
                    break
                edges = False

            if len(last_points):
                del plan[vid]
                if not edges:
                    plan[vid:vid] = last_points

        return plan

    def plan(self, start_config, goal_config, eta=0.5, max_iterations=10000, bias_thresh=0.05, second_try=False):

        if 'adaptive' == eta:
            adaptive = True
            eta = 0.5
        else:
            adaptive = False

        # Initialize an empty plan
        plan = []

        # Start with adding the start configuration to the tree
        self.tree.add_vertex(start_config)

        goal_id = -1
        streak = 0
        # Main search loop
        for step in range(max_iterations):
            x_rand = self.sample_random_config(goal_pose=(goal_config if np.random.uniform() < bias_thresh else None))
            near_id, x_near = self.tree.get_nearest_vertex(x_rand)
            x_new = self.extend_and_check(x_near, x_rand, eta)
            if x_new is not None:
                new_id = self.tree.add_vertex(x_new)
                self.tree.add_edge(near_id, new_id)
                if adaptive:
                    if streak > 0:
                        streak = -1
                    elif -10 < streak <= 0:
                        streak -= 1
                    else:
                        eta = 1.1 * eta
                        if eta > 1.0:
                            eta = 1.0
                        streak = 0
            else:
                if adaptive:
                    if streak < 0:
                        streak = 1
                    elif 0 <= streak < 100:
                        streak += 1
                        if not streak % 10:
                            eta = 0.9 * eta
                            if eta < 0.001:
                                eta = 0.001
                    else:
                        break
                continue

            if self.planning_env.check_edge_validity(x_new, goal_config):
                goal_id = self.tree.add_vertex(goal_config)
                self.tree.add_edge(new_id, goal_id)
                break

        if goal_id > 0:
            end_id = goal_id
            while end_id:
                end_v = self.tree.vertices[end_id]
                plan.insert(0, end_v)

                # Greedy smoothing to shorten path if possible
                start_id = self.tree.edges[end_id]
                while start_id:
                    new_start_id = self.tree.edges[start_id]
                    if self.planning_env.check_edge_validity(end_v, self.tree.vertices[new_start_id]):
                        start_id = new_start_id
                    else:
                        break
                end_id = start_id

            plan.insert(0, self.tree.vertices[0])
            plan = self.post_process(plan)
            return plan
        elif not goal_id:
            plan.append(goal_config)
            return plan
        else:
            # if second_try:
            #     return None
            # else:
            #     res = self.plan(start_config, goal_config, eta, max_iterations, bias_thresh, second_try=True)
            #     print(f'Res is {res if not res else True} on second try')
            #     return res
            return None


def evaluate(planning_env, start, goal, eta=0.5, bias=0.05):
    planner = RRTPlanner(planning_env)

    ts = time.time()
    plan = planner.plan(start, goal, eta=eta, bias_thresh=bias)
    tf = time.time()
    duration = tf - ts

    if plan is None:
        raise StopIteration('No path found')

    if plan[0] != start or plan[-1] != goal:
        raise StopIteration('Wrong path')

    total_cost = 0
    invalid_samples = 0
    total_samples = 0
    for segment in zip(plan, plan[1:]):
        first_config, second_config = segment
        segment_length = planning_env.compute_distance(first_config, second_config)
        total_cost += segment_length
        for _ in range(100 * int(np.ceil(segment_length))):
            a = np.random.uniform(0, 1)
            sample_x = a * first_config.x + (1 - a) * second_config.x
            sample_y = a * first_config.y + (1 - a) * second_config.y
            sample_config = Config(sample_x, sample_y)
            total_samples += 1
            if not planning_env.check_state_validity(sample_config):
                invalid_samples += 1
    # planning_env.visualize_plan(plan)
    invalidity_ratio = invalid_samples / total_samples
    return invalidity_ratio, total_cost, duration


if __name__ == '__main__':

    attempts = 100

    points = {'map1': [], 'map2': []}
    for map in ['map1', 'map2']:
        mapenv = MapEnvironment(f'{map}.txt')
        while len(points[map]) < attempts:
            x = np.random.uniform(*mapenv.xlimit, size=2)
            y = np.random.uniform(*mapenv.xlimit, size=2)
            start = Config(x[0], y[0])
            goal = Config(x[1], y[1])
            if not (mapenv.check_state_validity(start) and mapenv.check_state_validity(goal)):
                continue
            else:
                points[map].append((start, goal))

    for eta in reversed([0.01, 0.1, 0.25, 0.5, 'adaptive']):
    # for eta in [0.5]:
        for bias in [0.1, 0.05, 0.0]:
            print(f'Eta: {eta}, Bias: {bias}')
            print('=========================')
            for map in ('map1', 'map2'):
                mapenv = MapEnvironment(f'{map}.txt')

                high_invalid = 0
                mean_tot_c = 0
                mean_t = 0
                not_found = 0

                for start, goal in tqdm(points[map]):
                    try:
                        invalidity_ratio, total_cost, duration = evaluate(mapenv, start=start, goal=goal, eta=eta, bias=bias)
                    except StopIteration:
                        not_found += 1
                        continue

                    if invalidity_ratio > 0.05:
                        high_invalid += 1
                        continue
                    mean_tot_c += total_cost
                    mean_t += duration

                div = attempts - not_found - high_invalid

                print(f'Env: {map}.txt, Not found: {not_found}, Invalidity: {high_invalid}, Cost: {mean_tot_c / div}, Duration: {mean_t / div}')