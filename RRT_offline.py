import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import json

import sys

arg = sys.argv

if len(arg) < 3:
    print('Insufficient params. Correct usage: python <script> <path to data file> <path to save controls>')
    sys.exit(0)

# init constants
dt = 0.1
velocity_range = (-5, 5)
acceleration_range = (-10, 10)
car_dims = (2.385, 1.2)
car_axis_margins = (1.98, 0.6)
time_range_scope = 40
robot_xy = (100, 5)
target_s = 27
initial_s = 5

# init trackers
all_states = []
collision_states = {}

# load cars' positions
cars_positions = json.load(open(arg[1]))
cars_position_at_t = {}
for key in cars_positions:
    pos = np.array(cars_positions[key])
    for x, y, d, t in pos:
        t = round(t, 1)
        if cars_position_at_t.get(t) is None:
            cars_position_at_t[t] = [(x, y, d)]
        else:
            cars_position_at_t[t].append((x, y, d))


def create_roadmap(starting_pos, target, N):
    global time_range_scope

    # init state trackers
    final_states = []
    controls_input = {}

    # init graph
    graph = nx.DiGraph()
    graph.add_node((starting_pos, 0, 0))

    i = 0
    n_resamples = 0
    n_collisions = 0
    n_constraints = 0
    while i < N:
        # select node to expand from graph
        m_g = select_milestone(graph)

        # sample new milestone
        m_new, u = sample_new_milestone(m_g)

        # store the new state for analysis
        all_states.append(m_new)

        # check if dynamic constraints satisfied
        if (not velocity_range[0] <= m_new[1] <= velocity_range[1]) or m_new[0] < m_g[0] and m_new[0] < 10:
            n_constraints += 1
            continue

        # check if m_new already exists in the graph
        if graph.has_node(m_new):
            n_resamples += 1
            continue

        # check if m_new in collision state from cache
        if collision_states.get(m_new, 0) == 1:
            n_collisions += 1
            continue

        # if m_new not found in collision cache, check if it is collision free
        is_free_trajectory = propagate(m_g, m_new)

        # if collision free, add to graph
        if is_free_trajectory:

            # add node to graph
            graph.add_weighted_edges_from([(m_g, m_new, dt)])

            # add the control associated with the edge
            controls_input[(m_g, m_new)] = u

            # if m_new is reaches final state, add it to list of final states
            if m_new[0] >= target:
                print('At i: ', i, ' Final achieved', round(m_new[0], 5), round(m_new[1], 2), round(m_new[2], 1))

                # add to list of final states
                final_states.append(m_new)

                # decrease the scope of sampling to find better solution than current solution
                if m_new[2] < time_range_scope:
                    time_range_scope = m_new[2]

            i += 1
        else:
            n_collisions += 1
            collision_states[m_new] = 1
            continue

    # print metrics
    print('Total Iterations: ', (i + n_collisions + n_constraints + n_resamples))
    print('n_collisions: ', n_collisions)
    print('n_constraints: ', n_constraints)
    print('n_resamples: ', n_resamples)

    # if final state is empty, then no feasible path found
    if len(final_states) == 0:
        print('Path not found')
        return None, None, None
    return graph, final_states, controls_input


def select_milestone(graph):
    # generate random parameters
    s = np.random.rand() * target_s
    t = np.random.rand() * time_range_scope

    # get nearest existing milestone
    nodes = np.array(list(graph.nodes()))
    m_g = np.argmin(np.sum((nodes[:, [0, 2]] - (s, t)) ** 2, axis=1))
    return tuple(nodes[m_g])


def sample_new_milestone(milestone):
    s, v, t = milestone

    # choose a control
    acc = choose_control()

    # calculate new state
    _v = v + acc * dt
    _s = s + v * dt + acc * dt * dt / 2
    _t = t + dt

    return (_s, _v, _t), acc


def choose_control():
    acc_min, acc_max = acceleration_range

    # create integer valued accelerations
    acc = np.arange(acc_min, acc_max + 1, dtype=float)

    # biasing acceleration to reach goal state faster
    # p = np.array([1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 5])
    p = np.array([1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7])
    p = p / np.sum(p)

    # biased sampling
    acc = np.random.choice(acc, p=p)
    return acc


def vertical_collision(y1, y2, s1, s2, v2):
    # robot's velocity to model area of imminent collision as well
    y1 = y1 - car_axis_margins[1] - car_axis_margins[0] - np.abs(v2 * v2 / 2 / (acceleration_range[0])) - 0.25
    y2 = y2 + car_axis_margins[1] + (car_dims[0] - car_axis_margins[0])

    # basically check if robot's position is out of collision area
    if (s1 > y2 and s2 > y2) or (s1 < y1 and s2 < y1):
        return False
    else:
        return True


def horizontal_collision(x1, x2, d):
    v = (x2 - x1) / dt
    v = np.abs(v)

    # adding extra padding of 1 timestep before and after to a
    if d == 0:
        x1 = x1 - (car_dims[0] - car_axis_margins[0]) - car_axis_margins[
            1] - v * dt  # extra padding because of lags in ros
        x2 = x2 + car_axis_margins[0] + car_axis_margins[1] + 2 * v * dt
        if x1 < robot_xy[0] < x2:
            return True
    else:
        x1 = x1 + (car_dims[0] - car_axis_margins[0]) + car_axis_margins[1] + v * dt
        x2 = x2 - car_axis_margins[0] - car_axis_margins[1] - 2 * v * dt
        if x2 < robot_xy[0] < x1:
            return True

    return False


def propagate(old_m, new_m):
    s1, v1, t1, = old_m
    s2, v2, t2 = new_m

    car_positions_t1 = cars_position_at_t[round(t1, 1)]
    car_positions_t2 = cars_position_at_t[round(t2, 1)]

    for (x1, y1, d1), (x2, y2, d2) in zip(car_positions_t1, car_positions_t2):
        x_collide = horizontal_collision(x1, x2, d)
        y_collide = vertical_collision(y1, y2, s1, s2, v2)

        if x_collide and y_collide:
            return False
    return True


initial = (initial_s, 0, 0)

print('Starting RRT from position: ', initial)
roadmap, final_states, controls_input = create_roadmap(initial_s, target_s, 5000)

# plot all nodes and collision states
plt.plot(np.array(all_states)[:, 2], np.array(all_states)[:, 0], 'go', ms=1, label='Sampled points')
if len(collision_states) > 0:
    plt.plot(np.array(list(collision_states.keys()))[:, 2], np.array(list(collision_states.keys()))[:, 0], 'ro', ms=2,
             label='Collision points')

# if roadmap is found, compute the shortest path to goal state
if roadmap is not None:
    final = final_states[0]
    shortest_t = final[2]
    shortest_d = nx.shortest_path_length(roadmap, source=initial, target=final)

    # get the fastest reached final position
    for node in final_states:
        t = node[2]
        if t < shortest_t or (
                        t == shortest_t and nx.shortest_path_length(roadmap, source=initial, target=node) < shortest_d):
            shortest_t = t
            shortest_d = nx.shortest_path_length(roadmap, source=initial, target=node)
            final = node
    path = nx.shortest_path(roadmap, source=initial, target=final)

    # get the controls corresponding to shortest path
    control_list = []
    for i in range(len(path) - 1):
        control_list.append((controls_input[path[i], path[i + 1]], path[i][2]))

    # dump commands to json if user says yes
    robot_commands = {
        'robot_0': control_list
    }

    a = input('Do you want to save this action map?')
    if a == 'y' or a == 'Y':
        # dump controls to output file
        json.dump(robot_commands, open(arg[2], 'w'))
    plt.plot(np.array(path)[:, 2], np.array(path)[:, 0], 'b-', ms=2, label='Chosen path')

plt.ylabel('distance')
plt.xlabel('time')
plt.legend()
plt.show()
