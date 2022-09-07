import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[-2.01546, -5.83923, 4.0, 0.07213],
                        [-1.73089, -5.88578, 4.0, 0.07209],
                        [-1.44593, -5.9257, 4.0, 0.07193],
                        [-1.16116, -5.95875, 4.0, 0.07167],
                        [-0.87713, -5.98462, 4.0, 0.0713],
                        [-0.59442, -6.00289, 4.0, 0.07082],
                        [-0.31361, -6.01311, 3.95633, 0.07103],
                        [-0.03528, -6.01478, 3.77953, 0.07364],
                        [0.23997, -6.00734, 3.61908, 0.07608],
                        [0.51153, -5.99023, 3.47638, 0.07827],
                        [0.77877, -5.96291, 3.35253, 0.08013],
                        [1.04107, -5.92484, 3.24849, 0.08159],
                        [1.29778, -5.8755, 3.16513, 0.08259],
                        [1.54826, -5.81446, 3.10334, 0.08308],
                        [1.7919, -5.74134, 3.06398, 0.08302],
                        [2.02807, -5.65583, 3.04797, 0.08241],
                        [2.25621, -5.55773, 3.04797, 0.08148],
                        [2.47578, -5.44695, 3.04797, 0.08069],
                        [2.6863, -5.3235, 3.04797, 0.08007],
                        [2.88738, -5.18749, 3.04797, 0.07964],
                        [3.0787, -5.03918, 3.05624, 0.07921],
                        [3.26006, -4.87893, 3.08985, 0.07833],
                        [3.43135, -4.70721, 3.15002, 0.077],
                        [3.59261, -4.52459, 3.23831, 0.07523],
                        [3.744, -4.33176, 3.35682, 0.07303],
                        [3.88579, -4.12949, 3.50864, 0.0704],
                        [4.01842, -3.91862, 3.69839, 0.06736],
                        [4.14242, -3.70004, 3.93332, 0.06389],
                        [4.25847, -3.4747, 4.0, 0.06337],
                        [4.36734, -3.24358, 4.0, 0.06387],
                        [4.46992, -3.00763, 4.0, 0.06432],
                        [4.56718, -2.7678, 4.0, 0.0647],
                        [4.66018, -2.52502, 4.0, 0.065],
                        [4.75002, -2.28013, 4.0, 0.06521],
                        [4.83755, -2.03376, 4.0, 0.06536],
                        [4.92291, -1.79838, 4.0, 0.06259],
                        [5.01017, -1.56421, 4.0, 0.06248],
                        [5.09973, -1.33149, 4.0, 0.06234],
                        [5.19208, -1.10053, 4.0, 0.06219],
                        [5.28762, -0.87156, 4.0, 0.06202],
                        [5.3866, -0.64474, 4.0, 0.06187],
                        [5.48908, -0.42008, 4.0, 0.06173],
                        [5.59493, -0.1975, 4.0, 0.06162],
                        [5.70385, 0.02319, 4.0, 0.06153],
                        [5.81534, 0.24231, 4.0, 0.06146],
                        [5.92889, 0.46017, 4.0, 0.06142],
                        [6.04413, 0.67698, 4.0, 0.06139],
                        [6.16074, 0.8928, 4.0, 0.06133],
                        [6.2764, 1.10373, 4.0, 0.06014],
                        [6.3904, 1.31478, 3.55859, 0.06741],
                        [6.50159, 1.52629, 3.10026, 0.07708],
                        [6.60827, 1.73853, 2.73794, 0.08676],
                        [6.70852, 1.95176, 2.38933, 0.09861],
                        [6.80038, 2.16618, 2.08387, 0.11194],
                        [6.88143, 2.38195, 2.0, 0.11524],
                        [6.94867, 2.59912, 2.0, 0.11367],
                        [6.99878, 2.81747, 2.0, 0.11202],
                        [7.02716, 3.03634, 2.0, 0.11035],
                        [7.02797, 3.25393, 2.0, 0.10879],
                        [6.9999, 3.46753, 2.16293, 0.09961],
                        [6.94792, 3.6757, 2.31749, 0.09258],
                        [6.87562, 3.87766, 2.45776, 0.08728],
                        [6.78559, 4.07291, 2.61487, 0.08222],
                        [6.68018, 4.26129, 2.71075, 0.07963],
                        [6.56066, 4.44243, 2.78986, 0.07779],
                        [6.42803, 4.61595, 2.84231, 0.07684],
                        [6.28297, 4.78137, 2.86412, 0.07682],
                        [6.126, 4.93812, 2.86412, 0.07745],
                        [5.95728, 5.0853, 2.86934, 0.07803],
                        [5.77761, 5.22254, 2.86934, 0.07879],
                        [5.58689, 5.34854, 2.8771, 0.07945],
                        [5.38563, 5.46242, 2.90805, 0.07952],
                        [5.17458, 5.56351, 2.95201, 0.07927],
                        [4.95467, 5.65126, 3.00355, 0.07883],
                        [4.72688, 5.72528, 3.06016, 0.07827],
                        [4.49224, 5.7853, 3.12119, 0.0776],
                        [4.25173, 5.83115, 3.18722, 0.07682],
                        [4.00633, 5.8628, 3.2597, 0.07591],
                        [3.75695, 5.88035, 3.34071, 0.07483],
                        [3.50442, 5.88398, 3.43286, 0.07357],
                        [3.24952, 5.87402, 3.53919, 0.07208],
                        [2.99291, 5.85091, 3.66326, 0.07033],
                        [2.73517, 5.8152, 3.80925, 0.06831],
                        [2.47679, 5.76755, 3.98222, 0.06598],
                        [2.21814, 5.70871, 4.0, 0.06631],
                        [1.9595, 5.63954, 4.0, 0.06693],
                        [1.70108, 5.56095, 4.0, 0.06753],
                        [1.44295, 5.47394, 4.0, 0.0681],
                        [1.18516, 5.37956, 4.0, 0.06863],
                        [0.92768, 5.27894, 4.0, 0.06911],
                        [0.67045, 5.1733, 4.0, 0.06952],
                        [0.4134, 5.06388, 4.0, 0.06984],
                        [0.15647, 4.95161, 4.0, 0.0701],
                        [-0.10039, 4.8373, 4.0, 0.07029],
                        [-0.36678, 4.71711, 4.0, 0.07306],
                        [-0.6337, 4.5982, 4.0, 0.07305],
                        [-0.90115, 4.48057, 4.0, 0.07304],
                        [-1.16916, 4.36431, 4.0, 0.07304],
                        [-1.43781, 4.24962, 4.0, 0.07303],
                        [-1.70714, 4.13655, 4.0, 0.07302],
                        [-1.97717, 4.02521, 4.0, 0.07302],
                        [-2.2479, 3.91558, 4.0, 0.07302],
                        [-2.51929, 3.80753, 4.0, 0.07303],
                        [-2.79123, 3.70085, 4.0, 0.07303],
                        [-3.06359, 3.59517, 4.0, 0.07304],
                        [-3.32879, 3.49311, 4.0, 0.07104],
                        [-3.59046, 3.39008, 4.0, 0.07031],
                        [-3.84781, 3.28506, 4.0, 0.06949],
                        [-4.10066, 3.17694, 4.0, 0.06875],
                        [-4.34883, 3.06476, 3.98776, 0.0683],
                        [-4.59193, 2.9477, 3.77734, 0.07143],
                        [-4.82937, 2.82503, 3.60431, 0.07415],
                        [-5.06048, 2.69607, 3.46514, 0.07638],
                        [-5.2845, 2.56014, 3.35568, 0.07809],
                        [-5.50067, 2.41665, 3.27193, 0.0793],
                        [-5.70818, 2.26507, 3.21044, 0.08004],
                        [-5.90626, 2.10497, 3.16823, 0.08039],
                        [-6.09417, 1.93607, 3.14262, 0.0804],
                        [-6.27122, 1.75822, 3.13097, 0.08015],
                        [-6.43677, 1.57144, 3.13064, 0.07972],
                        [-6.59021, 1.37585, 3.13064, 0.07941],
                        [-6.73103, 1.17173, 3.13064, 0.07921],
                        [-6.85872, 0.95948, 3.13064, 0.07912],
                        [-6.97289, 0.73962, 3.13064, 0.07913],
                        [-7.07316, 0.51279, 3.13893, 0.07901],
                        [-7.15922, 0.2797, 3.15314, 0.0788],
                        [-7.23084, 0.04116, 3.17068, 0.07855],
                        [-7.28779, -0.20196, 3.18912, 0.0783],
                        [-7.32993, -0.44872, 3.20637, 0.07807],
                        [-7.35714, -0.69815, 3.22074, 0.07791],
                        [-7.36936, -0.94925, 3.22787, 0.07788],
                        [-7.36654, -1.20102, 3.21975, 0.0782],
                        [-7.34871, -1.45242, 3.21171, 0.07847],
                        [-7.3159, -1.70245, 3.20573, 0.07866],
                        [-7.2682, -1.95012, 3.20388, 0.07872],
                        [-7.20572, -2.19446, 3.20388, 0.07872],
                        [-7.12864, -2.43456, 3.20388, 0.07871],
                        [-7.03715, -2.66954, 3.20388, 0.07871],
                        [-6.93149, -2.89859, 3.20388, 0.07873],
                        [-6.81197, -3.12097, 3.20826, 0.07869],
                        [-6.67892, -3.33602, 3.22087, 0.07851],
                        [-6.53272, -3.54316, 3.24357, 0.07817],
                        [-6.37381, -3.74191, 3.27807, 0.07763],
                        [-6.20266, -3.93186, 3.32585, 0.07688],
                        [-6.01978, -4.11274, 3.3882, 0.07591],
                        [-5.82575, -4.28435, 3.46615, 0.07473],
                        [-5.62115, -4.4466, 3.56053, 0.07334],
                        [-5.4066, -4.59948, 3.67186, 0.07175],
                        [-5.18276, -4.74309, 3.80039, 0.06998],
                        [-4.95029, -4.8776, 3.94595, 0.06807],
                        [-4.70986, -5.00324, 4.0, 0.06782],
                        [-4.46217, -5.12029, 4.0, 0.06849],
                        [-4.20789, -5.22908, 4.0, 0.06914],
                        [-3.94771, -5.32995, 4.0, 0.06976],
                        [-3.68228, -5.42325, 4.0, 0.07034],
                        [-3.41228, -5.50929, 4.0, 0.07085],
                        [-3.13833, -5.58838, 4.0, 0.07128],
                        [-2.86107, -5.66075, 4.0, 0.07164],
                        [-2.5811, -5.7266, 4.0, 0.0719],
                        [-2.29903, -5.78601, 4.0, 0.07207]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 19
        FASTEST_TIME = 15
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 19  # seconds (time that is easily done by model)
        FASTEST_TIME = 15  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward(verbose=True) # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)