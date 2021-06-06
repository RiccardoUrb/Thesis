import time
import os
import numpy as np
from os.path import join
from map_class import Map
import animation
from drone_class import Drone
from keras.models import load_model


def make_action_cycle(drone_list):
    for drone_obj in drone_list:
        drone_obj.action(drone_list)

def get_exploration_percentage(drone_list):
    nonzero_elements = np.count_nonzero(drone_list[0].covered_area_matrix)
    total_elements = drone_list[0].covered_area_matrix.size
    percentage = nonzero_elements / total_elements
    return percentage

def main():
    begin_time = time.time()
    parent_folder_path = os.path.dirname(os.getcwd())

    # setup for animation
    max_iterations = 100

    # RL model
    observation_size = 25
    path_planning_model_number = 10
    # coverage_model_number = 1
    path_planning_model_dir = join(parent_folder_path, 'models', 'actor_' + str(path_planning_model_number) + '.h5')
    # coverage_model_dir = join(parent_folder_path, 'models', 'coverage_actor_' + str(coverage_model_number) + '.h5')

    # generate world
    world = Map()
    world.x_dimension = 20
    world.y_dimension = 10
    world.z_dimension = 3
    world.x_resolution = 0.01
    world.y_resolution = 0.01
    world.z_resolution = 0.01

    # set up drone swarm
    n_drones = 1
    drone_selection_index = 0  # chosen drone that will be visualize in the animation
    drone_id = 0
    drone_list = []
    drone_max_speed = 0.15
    drone_safe_distance = 2.3  # dimension of mobile obstacles for other drones
    fixed_obstacles_safe_distance = 1.1
    minima_osbtacle_dimension = 3.5
    mobile_peak_value, fixed_peak_value, minima_peak_value = 500, 50, 30  # case study
    attractive_constant = 70
    max_steps_apf_descent = 5
    vision_range = 2  # [m] max distance at the drone can detect obstacles
    max_vision_angle = 45 # half cone of vision
    angle_resolution = 50  # number of precision points for vision
    predict_length = 10
    min_obstacle_distance = 0.35
    max_steps_with_single_goal = 200

    for i in range(0, n_drones):
        drone = Drone()
        drone_id += 1
        drone.set_drone_ID(drone_id)
        drone.import_map_properties(world)
        drone.set_drone_safe_distance(drone_safe_distance)
        drone.set_fixed_obstacles_safe_distance(fixed_obstacles_safe_distance)
        drone.set_minima_obstacles_dimension(minima_osbtacle_dimension)
        drone.set_matrix_peak_value(mobile_peak_value, fixed_peak_value, minima_peak_value)
        drone.set_attractive_constant(attractive_constant)
        drone.set_predict_length(predict_length)
        drone.max_steps_apf_descent_path_planning(max_steps_apf_descent)
        drone.set_max_speed(drone_max_speed)
        drone.min_obstacles_distance(min_obstacle_distance)
        drone.set_vision_settings(vision_range, max_vision_angle, angle_resolution)
        drone.set_RL_path_planning_model(load_model(path_planning_model_dir, compile=False))
        # drone.set_RL_coverage_model(load_model(coverage_model_dir, compile=False))
        drone.set_max_steps_with_single_goal(max_steps_with_single_goal)
        drone.set_observation_size(observation_size)
        drone.set_initial_position([8, 8.5, 2])
        drone.initialize(n_drones)
        drone.set_initial_goal([1, 1, 2])
        drone_list.append(drone)
        print(drone)

    # lidar mode for the first iteration of the simulation
    # for drone_obj in drone_list:
    #     drone_obj.lidar_operation_mode()
    #     drone_obj.detect_obstacles()
    #     # drone_obj.generate_border_obstacles()
    #     drone_obj.share_covered_area(drone_list)
    #     drone_obj.update_fixed_obstacles(drone_list)
    #     drone_obj.normal_operation_mode()

    # main cycle of animation
    simulation_steps = 0
    elapsed_time = round((time.time() - begin_time), 3)
    print('Time elapsed for initialization: ' + str(elapsed_time) + ' s')
    print('SIMULATION IS STARTED')
    while simulation_steps <= max_iterations:
        make_action_cycle(drone_list)
        # exploration_percentage = get_exploration_percentage(drone_list)
        # print("Explored map percentage = {:.2%} | Step N° {}".format(exploration_percentage, simulation_steps))
        print('drone position:', np.round(drone_list[0].position, 2), '  goal position:', np.round(drone_list[0].goal, 2))
        simulation_steps += 1
    animation.plot_figures(drone_list)

if __name__ == "__main__":
    main()
