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
        racing_track = [[-2.03762, -5.95555, 4.0, 0.07516],
                            [-1.7405, -6.00582, 4.0, 0.07534],
                            [-1.44228, -6.05404, 4.0, 0.07552],
                            [-1.14458, -6.09961, 4.0, 0.07529],
                            [-0.84789, -6.1415, 3.97075, 0.07546],
                            [-0.55242, -6.17856, 3.6877, 0.08075],
                            [-0.25845, -6.20961, 3.38105, 0.08743],
                            [0.0337, -6.23347, 3.21397, 0.0912],
                            [0.32365, -6.24897, 3.02921, 0.09586],
                            [0.61097, -6.25494, 2.90342, 0.09898],
                            [0.89513, -6.2502, 2.7314, 0.10405],
                            [1.17539, -6.23316, 2.61523, 0.10736],
                            [1.45113, -6.20302, 2.5273, 0.10975],
                            [1.72154, -6.1587, 2.46318, 0.11125],
                            [1.98587, -6.09959, 2.42027, 0.11191],
                            [2.24296, -6.02444, 2.39628, 0.11178],
                            [2.49172, -5.93255, 2.38954, 0.11098],
                            [2.73102, -5.82352, 2.38954, 0.11005],
                            [2.9597, -5.69724, 2.38954, 0.10933],
                            [3.17668, -5.55388, 2.38954, 0.10883],
                            [3.38091, -5.39391, 2.38954, 0.10857],
                            [3.57145, -5.21801, 2.39924, 0.10808],
                            [3.74754, -5.02709, 2.42567, 0.10708],
                            [3.9086, -4.82219, 2.47033, 0.1055],
                            [4.05431, -4.60449, 2.53615, 0.10329],
                            [4.18464, -4.37527, 2.62778, 0.10035],
                            [4.29992, -4.13582, 2.75234, 0.09655],
                            [4.40081, -3.88748, 2.92106, 0.09176],
                            [4.48835, -3.63159, 3.1511, 0.08583],
                            [4.56398, -3.36945, 3.45342, 0.079],
                            [4.62934, -3.10232, 3.82819, 0.07184],
                            [4.6861, -2.83124, 3.70512, 0.07475],
                            [4.73512, -2.55683, 3.46897, 0.08036],
                            [4.77716, -2.27956, 3.21189, 0.08731],
                            [4.8107, -2.01486, 2.98751, 0.08931],
                            [4.8516, -1.75415, 2.84203, 0.09286],
                            [4.90077, -1.498, 2.7643, 0.09435],
                            [4.95904, -1.24696, 2.75543, 0.09353],
                            [5.02752, -1.00174, 2.75543, 0.0924],
                            [5.10736, -0.76311, 2.75543, 0.09132],
                            [5.1993, -0.53164, 2.75543, 0.09039],
                            [5.30366, -0.30763, 2.75543, 0.08969],
                            [5.42024, -0.09108, 2.82055, 0.08719],
                            [5.54832, 0.11834, 2.97342, 0.08256],
                            [5.6867, 0.32131, 3.23464, 0.07594],
                            [5.83378, 0.51875, 3.61706, 0.06807],
                            [5.98788, 0.71171, 3.34355, 0.07385],
                            [6.14763, 0.90101, 3.04653, 0.08131],
                            [6.31231, 1.08711, 2.72931, 0.09105],
                            [6.48178, 1.28722, 2.52395, 0.1039],
                            [6.64346, 1.4902, 2.33499, 0.11114],
                            [6.79461, 1.69685, 2.1843, 0.11721],
                            [6.93344, 1.9075, 2.08351, 0.12109],
                            [7.0574, 2.12259, 2.02475, 0.12261],
                            [7.16453, 2.34217, 2.0, 0.12216],
                            [7.25261, 2.56617, 2.0, 0.12035],
                            [7.3195, 2.79428, 2.0, 0.11886],
                            [7.36356, 3.02579, 2.0, 0.11784],
                            [7.38367, 3.25971, 2.0, 0.11739],
                            [7.37922, 3.49473, 2.00309, 0.11735],
                            [7.35004, 3.72937, 2.0285, 0.11656],
                            [7.29639, 3.96201, 2.07095, 0.11528],
                            [7.21888, 4.19093, 2.1254, 0.11372],
                            [7.11838, 4.41443, 2.1872, 0.11204],
                            [6.99604, 4.63085, 2.25237, 0.11038],
                            [6.85314, 4.8386, 2.3177, 0.10879],
                            [6.69112, 5.03622, 2.38087, 0.10733],
                            [6.51148, 5.22238, 2.44042, 0.10601],
                            [6.31575, 5.3959, 2.49563, 0.10481],
                            [6.10551, 5.55576, 2.54629, 0.10373],
                            [5.88231, 5.70111, 2.59246, 0.10274],
                            [5.64769, 5.83121, 2.63426, 0.10184],
                            [5.40314, 5.9455, 2.67172, 0.10103],
                            [5.15016, 6.04351, 2.70472, 0.10031],
                            [4.89017, 6.12489, 2.73313, 0.09968],
                            [4.62458, 6.1894, 2.75699, 0.09913],
                            [4.35474, 6.23686, 2.7768, 0.09867],
                            [4.08194, 6.26718, 2.79384, 0.09824],
                            [3.80741, 6.28037, 2.81038, 0.0978],
                            [3.53229, 6.2765, 2.82977, 0.09723],
                            [3.25763, 6.25576, 2.85644, 0.09643],
                            [2.98433, 6.21849, 2.89576, 0.09525],
                            [2.71316, 6.16514, 2.9539, 0.09356],
                            [2.44472, 6.09638, 3.03782, 0.09122],
                            [2.17941, 6.01304, 3.15566, 0.08813],
                            [1.91745, 5.91615, 3.31714, 0.0842],
                            [1.65885, 5.80694, 3.53388, 0.07943],
                            [1.40346, 5.68677, 3.81953, 0.0739],
                            [1.15096, 5.55713, 4.0, 0.07096],
                            [0.90094, 5.41945, 4.0, 0.07136],
                            [0.653, 5.27489, 4.0, 0.07175],
                            [0.40678, 5.12422, 4.0, 0.07217],
                            [0.16187, 4.96795, 4.0, 0.07263],
                            [-0.08985, 4.81387, 4.0, 0.07378],
                            [-0.34416, 4.66501, 4.0, 0.07367],
                            [-0.60108, 4.52183, 4.0, 0.07353],
                            [-0.86084, 4.38501, 4.0, 0.0734],
                            [-1.12349, 4.25486, 4.0, 0.07328],
                            [-1.38911, 4.13174, 4.0, 0.07319],
                            [-1.65792, 4.01631, 4.0, 0.07314],
                            [-1.92995, 3.90879, 4.0, 0.07313],
                            [-2.20507, 3.80905, 4.0, 0.07316],
                            [-2.48302, 3.71653, 4.0, 0.07323],
                            [-2.76334, 3.6302, 4.0, 0.07333],
                            [-3.04567, 3.54917, 4.0, 0.07343],
                            [-3.32978, 3.47295, 4.0, 0.07354],
                            [-3.61553, 3.40115, 3.86599, 0.07621],
                            [-3.9029, 3.33389, 3.52059, 0.08383],
                            [-4.17938, 3.26388, 3.18171, 0.08964],
                            [-4.45256, 3.18822, 3.04206, 0.09318],
                            [-4.72163, 3.10576, 2.81351, 0.10002],
                            [-4.98559, 3.01511, 2.67899, 0.10418],
                            [-5.24339, 2.9149, 2.59259, 0.10668],
                            [-5.49358, 2.80341, 2.54293, 0.10771],
                            [-5.7354, 2.68006, 2.52257, 0.10761],
                            [-5.96731, 2.54337, 2.52257, 0.10672],
                            [-6.1881, 2.39257, 2.52257, 0.10599],
                            [-6.39673, 2.2273, 2.52257, 0.10551],
                            [-6.59227, 2.04759, 2.52257, 0.10528],
                            [-6.77398, 1.85376, 2.52624, 0.10517],
                            [-6.94124, 1.64644, 2.54971, 0.10447],
                            [-7.09357, 1.42645, 2.58905, 0.10335],
                            [-7.23064, 1.19479, 2.64034, 0.10195],
                            [-7.35222, 0.95259, 2.69957, 0.10038],
                            [-7.45817, 0.70109, 2.76265, 0.09879],
                            [-7.54846, 0.44154, 2.82549, 0.09726],
                            [-7.62308, 0.17524, 2.8842, 0.09589],
                            [-7.68208, -0.09651, 2.93526, 0.09474],
                            [-7.72552, -0.37245, 2.97572, 0.09387],
                            [-7.75345, -0.65133, 2.97759, 0.09413],
                            [-7.76591, -0.93193, 2.94243, 0.09546],
                            [-7.76293, -1.21304, 2.90038, 0.09693],
                            [-7.74449, -1.49349, 2.85442, 0.09846],
                            [-7.71056, -1.77209, 2.80768, 0.09996],
                            [-7.66111, -2.0477, 2.76331, 0.10133],
                            [-7.5961, -2.31913, 2.72439, 0.10245],
                            [-7.51551, -2.58524, 2.69395, 0.10321],
                            [-7.41932, -2.84488, 2.67483, 0.10351],
                            [-7.3076, -3.0969, 2.66976, 0.10326],
                            [-7.18047, -3.34022, 2.66976, 0.10283],
                            [-7.03813, -3.57381, 2.66976, 0.10246],
                            [-6.88087, -3.79673, 2.66976, 0.10218],
                            [-6.70911, -4.00815, 2.66976, 0.10203],
                            [-6.52336, -4.20738, 2.68128, 0.10159],
                            [-6.32424, -4.39393, 2.71181, 0.10062],
                            [-6.11249, -4.56748, 2.76366, 0.09907],
                            [-5.88893, -4.72793, 2.83916, 0.09692],
                            [-5.65446, -4.87541, 2.94077, 0.09419],
                            [-5.41005, -5.01025, 3.07131, 0.09089],
                            [-5.1567, -5.13301, 3.23425, 0.08705],
                            [-4.8954, -5.24442, 3.43412, 0.08271],
                            [-4.62717, -5.34535, 3.6772, 0.07794],
                            [-4.35296, -5.43682, 3.97266, 0.07276],
                            [-4.07369, -5.51991, 4.0, 0.07284],
                            [-3.79022, -5.59576, 4.0, 0.07336],
                            [-3.50333, -5.66551, 4.0, 0.07381],
                            [-3.21377, -5.73036, 4.0, 0.07418],
                            [-2.92203, -5.79111, 4.0, 0.0745],
                            [-2.62855, -5.84851, 4.0, 0.07476],
                            [-2.33367, -5.90318, 4.0, 0.07498]]

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