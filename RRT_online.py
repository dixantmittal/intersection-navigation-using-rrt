import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import json

import sys

########## input commandline arguments ##########
arg = sys.argv
if len(arg) < 3:
    print('Insufficient params. Correct usage: python <script> <path to data file> <path to save controls>')
    sys.exit(0)

########## hyperparameters ##########
# > To control how much you want to depend on your heuristics and how concrete should be the heuristics
# > You would want to have a good heuristics, so initial n_samples should be high
# > increasing the decreased sample rate will increase the xomputation time but reduce the dependency on heuristics
# > reduced samples will result in failure to return new path, hence planner will depend on the heuristics

initial_n_samples = 15000
decrease_samples_after_steps = 1
decreased_sample_rate = 1000

########## init constants ##########
dt = 0.1
velocity_range = (-5, 5)
acceleration_range = (-10, 10)
car_dims = (2.385, 1.2)
car_axis_margins = (1.98, 0.6)
time_range_scope = 40
robot_xy = (100, 5)
target_s = 27
starting_pos = 5

########## load cars' actual positions ##########
cars_positions = json.load(open(arg[1]))
actual_cars_position_at_t = {}
for key in cars_positions:
    pos = np.array(cars_positions[key])
    for x, y, d, t in pos:
        t = round(t, 1)
        if actual_cars_position_at_t.get(t) is None:
            actual_cars_position_at_t[t] = [(x, y, d)]
        else:
            actual_cars_position_at_t[t].append((x, y, d))

car_position_model = {}

########## analysis Metrics ##########
all_sampled_states = []
collision_states = {}


########## define default backup plan ##########
def get_initial_backup_plan():
    acc = 0
    initial = (starting_pos, 0, 0)

    path = [initial]
    control = {}
    for i in range(time_range_scope):
        path.append(initial)
        control[(initial, initial)] = acc

    return path, control


########## start creating roadmap ##########
def create_roadmap():
    global time_range_scope

    # fix starting position
    initial = (starting_pos, 0, 0)

    roadmap = [initial]
    control_list = []

    backup_path, backup_action = get_initial_backup_plan()

    # initially set sampling factor high to find a good heuristic path
    n_samples = initial_n_samples

    # start travelling from initial position
    # Max hard limit = 30 seconds
    for i in range(300):

        # reset time range scope
        time_range_scope = 30

        # find the best path from current position
        print('Creating RRT from position: ', initial)
        graph, final_states, controls_input = create_future_roadmap(starting_state=initial, target=target_s,
                                                                    max_samples=n_samples)

        if graph is not None:
            final = final_states[0]
            shortest_t = final[2]
            shortest_d = nx.shortest_path_length(graph, source=initial, target=final)
            for node in final_states:
                t = node[2]
                if t < shortest_t or (
                                t == shortest_t and nx.shortest_path_length(graph, source=initial,
                                                                            target=node) < shortest_d):
                    shortest_t = t
                    shortest_d = nx.shortest_path_length(graph, source=initial, target=node)
                    final = node

            # path contains the path to follow from current node
            path = nx.shortest_path(graph, source=initial, target=final)
            if i > decrease_samples_after_steps:
                n_samples = decreased_sample_rate

        else:
            # if path is not found, follow the previous found path
            path = backup_path
            controls_input = backup_action

        # next milestone is next move in the path
        next_milestone = path[1]

        # add the corresponding control
        control_list.append(controls_input[(initial, next_milestone)])
        roadmap.append(next_milestone)

        # set the next starting position to the next milestone
        initial = next_milestone

        # if final position is reached, break
        if initial[0] > target_s:
            print('At i: ', i, ' Final achieved!')
            break

        # update the backup plan
        backup_path = path[1:]
        backup_action = controls_input

    # store the controls in the file
    temp = control_list
    control_list = []

    for i in range(len(roadmap) - 1):
        control_list.append((temp[i], roadmap[i][2]))

    robot_commands = {
        'robot_0': control_list
    }

    # dump controls to output file
    json.dump(robot_commands, open(arg[2], 'w'))

    # plot all nodes and collision states
    plt.plot(np.array(all_sampled_states)[:, 2], np.array(all_sampled_states)[:, 0], 'go', ms=1, label='Sampled points')
    if len(collision_states) > 0:
        plt.plot(np.array(list(collision_states.keys()))[:, 2], np.array(list(collision_states.keys()))[:, 0], 'ro',
                 ms=2,
                 label='Collision points')

    plt.plot(np.array(roadmap)[:, 2], np.array(roadmap)[:, 0], 'b-', ms=2, label='Chosen path')
    plt.ylabel('distance')
    plt.xlabel('time')
    plt.legend()
    plt.show()


def create_future_roadmap(starting_state, target, max_samples):
    global time_range_scope
    global car_position_model

    # init state trackers
    final_states = []
    controls_input = {}

    # init graph
    graph = nx.DiGraph()
    graph.add_node(starting_state)

    # for Metrics
    n_resamples = 0
    n_collisions = 0
    n_constraints = 0

    # add actual car positions till t to model
    car_position_model = {}
    for key in actual_cars_position_at_t.keys():
        if key <= round(starting_state[2], 1):
            car_position_model[key] = actual_cars_position_at_t[key]

    # start sampling
    for i in range(max_samples):

        # select node to expand from graph
        m_g = select_milestone(graph, starting_state)

        # sample new milestone
        m_new, u = sample_new_milestone(m_g)

        # store the new state for analysis
        all_sampled_states.append(m_new)

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

        # if not collision free, add to collision cache
        if not is_free_trajectory:
            n_collisions += 1
            collision_states[m_new] = 1
            continue

        # add node to graph
        graph.add_edge(m_g, m_new)

        # add the control associated with the edge
        controls_input[(m_g, m_new)] = u

        # if m_new is reaches final state, add it to list of final states
        if m_new[0] >= target:

            # add to list of final states
            final_states.append(m_new)

            # decrease the scope of sampling to find better solution than current solution
            if m_new[2] < time_range_scope:
                time_range_scope = m_new[2]

            break

    # print metrics
    print('Total Iterations: ', i)
    print('n_collisions: ', n_collisions)
    print('n_constraints: ', n_constraints)
    print('n_resamples: ', n_resamples)

    # if final state is empty, then no feasible path found
    if len(final_states) == 0:
        print('Path not found')
        return None, None, None
    return graph, final_states, controls_input


def select_milestone(graph, offsets):
    # generate random parameters shifting and scaling to offsets
    s = np.random.rand() * (target_s - offsets[0]) + offsets[0]
    t = np.random.rand() * (time_range_scope - offsets[2]) + offsets[1]

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
    y1 = y1 - car_axis_margins[1] - car_axis_margins[0] - np.abs(v2 * v2 / 2 / (acceleration_range[0])) - 0.5
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
        x1 = x1 - (car_dims[0] - car_axis_margins[0]) - car_axis_margins[1] - v * dt
        x2 = x2 + car_axis_margins[0] + car_axis_margins[1] + 2 * v * dt
        if x1 < robot_xy[0] < x2:
            return True
    else:
        x1 = x1 + (car_dims[0] - car_axis_margins[0]) + car_axis_margins[1] + v * dt
        x2 = x2 - car_axis_margins[0] - car_axis_margins[1] - 2 * v * dt
        if x2 < robot_xy[0] < x1:
            return True

    return False


def predict_cars_position_at_t(t):
    t_1, t_2, t_3 = t - dt, t - 2 * dt, t - 3 * dt
    car_positions_t = None
    if round(t, 1) == 0.1:
        car_positions_t_1 = np.array(car_position_model[round(t_1, 1)])
        acceleration = 0
        velocity_t_1 = 5
        pos = car_positions_t_1[:, 0] + velocity_t_1 * dt + acceleration * dt * dt / 2
        pos = np.array(pos).reshape(len(pos), 1)
        car_positions_t = np.concatenate((pos, car_positions_t_1[:, 1:]), axis=1)
    elif round(t, 1) == 0.2:
        car_positions_t_1 = np.array(car_position_model[round(t_1, 1)])
        car_positions_t_2 = np.array(car_position_model[round(t_2, 1)])
        acceleration = 0
        velocity_t_1 = (car_positions_t_1[:, 0] - car_positions_t_2[:, 0]) / dt
        pos = car_positions_t_1[:, 0] + velocity_t_1 * dt + acceleration * dt * dt / 2
        pos = np.array(pos).reshape(len(pos), 1)
        car_positions_t = np.concatenate((pos, car_positions_t_1[:, 1:]), axis=1)
    else:
        car_positions_t_1 = np.array(car_position_model[round(t_1, 1)])
        car_positions_t_2 = np.array(car_position_model[round(t_2, 1)])
        car_positions_t_3 = np.array(car_position_model[round(t_3, 1)])

        velocity_t_1 = (car_positions_t_1[:, 0] - car_positions_t_2[:, 0]) / dt
        velocity_t_2 = (car_positions_t_2[:, 0] - car_positions_t_3[:, 0]) / dt

        acceleration = (velocity_t_1 - velocity_t_2) / dt
        pos = car_positions_t_1[:, 0] + velocity_t_1 * dt + acceleration * dt * dt / 2
        pos = np.array(pos).reshape(len(pos), 1)
        car_positions_t = np.concatenate((pos, car_positions_t_1[:, 1:]), axis=1)

    return tuple(map(tuple, car_positions_t))


def propagate(old_m, new_m):
    s1, v1, t1, = old_m
    s2, v2, t2 = new_m

    car_positions_t1 = car_position_model[round(t1, 1)]
    car_positions_t2 = car_position_model.get(round(t2, 1), None)
    if car_positions_t2 is None:
        car_positions_t2 = predict_cars_position_at_t(t2)
        car_position_model[round(t2, 1)] = car_positions_t2

    for (x1, y1, d1), (x2, y2, d2) in zip(car_positions_t1, car_positions_t2):
        x_collide = horizontal_collision(x1, x2, d)
        y_collide = vertical_collision(y1, y2, s1, s2, v2)

        if x_collide and y_collide:
            return False
    return True


create_roadmap()
