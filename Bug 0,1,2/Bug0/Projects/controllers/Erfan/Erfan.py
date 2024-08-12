import sys
import math
import random
from controller import Robot

webots_path = 'D:\\Apps\\Webots\\lib\\controller\\python'
sys.path.append(webots_path)    
    
# Initialize the e-puck robot
robot = Robot()

# Initialize and configure motors
left_motor = robot.getDevice('left wheel motor')
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0)

right_motor = robot.getDevice('right wheel motor')
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0)

sampling_period = 1  # in milliseconds
gps_sensor = robot.getDevice("gps")
compass_sensor = robot.getDevice("compass")

# Enable GPS and compass sensors
gps_sensor.enable(sampling_period)
compass_sensor.enable(sampling_period)

# Initialize and enable sonar sensors of e-pock
sonar_sensors = [robot.getDevice('ps' + str(i)) for i in range(8)]
for sensor in sonar_sensors:
    sensor.enable(sampling_period)
    
    
# Perform initial steps
robot.step(1000)
initial_gps_reading = gps_sensor.getValues()
initial_compass_reading = compass_sensor.getValues()

# Define some parameters
NUM_TIMESTEPS = 100000
OBSTACLE_THRESHOLD = 1000
OBSTACLE_DETECTED = False
TARGET_X, TARGET_Y = 0,-0.5

# Movement variables
current_direction, previous_direction = '', ''

# Start moving the robot
left_motor.setVelocity(1)
right_motor.setVelocity(1)


# Calculate the robot's heading in degrees using compass values
def calculate_heading(compass_vals):
    radian_angle = math.atan2(compass_vals[1], compass_vals[0])
    degree_angle = (radian_angle - 1.5708) / math.pi * 180.0
    if degree_angle < 0.0:
        degree_angle += 360.0
    
    heading = 360 - degree_angle
    if heading > 360.0:
        heading -= 360.0
    return heading
    
# Check if the robot is on the vertical line from start to goal point   
def is_vertical_aligned(current_x):
    return abs(current_x ) < 0.05

# Check if the robot is on the horizontal line from start to goal point       
def is_horizontal_aligned(current_y):
    return abs(current_y - 0.5) < 0.05

# Calculate the Euclidean distance between current position and goal    
def calculate_distance(start_x, start_y, target_x, target_y):
    return math.sqrt((start_x - target_x)**2 + (start_y - target_y)**2)


def calculate_angle_to_target(current_x, current_y, target_x, target_y):
    return math.atan2(target_y - current_y, target_x - current_x)

# Function to calculate the angle difference
def get_angle_difference(robot_angle, target_angle):
    difference = target_angle - robot_angle
    while difference < -math.pi:
        difference += 2 * math.pi
    while difference > math.pi:
        difference -= 2 * math.pi
    return difference

def calc_teta():
    pos = gps_sensor.getValues()
    north = compass_sensor.getValues()
    robot_angle = math.atan2(north[0], north[1])
    target_angle = calculate_angle_to_target(pos[0], pos[2], TARGET_X, TARGET_Y)
    angle_difference = get_angle_difference(robot_angle, target_angle)
    return angle_difference

def toward_goal():
    for time_step in range(10000):
        angle_difference = calc_teta()
        # Turn towards the target
        if abs(angle_difference) > 0.1:  # Threshold to avoid oscillations
            turn_speed = 6.0 if angle_difference > 0 else -6.0
            return -turn_speed, turn_speed
        else:
            return 1,1
        robot.step()

def is_obstacle_in_direction(theta, direction_angles, sensors):
    for angle, sensor_index in direction_angles:
        if abs(theta - angle) < 10 and sonar_sensors[sensor_index].getValue() < 80:
            return True
    return False

def has_clear_path(theta, current_y, target_y):
    is_moving_upwards = current_y > target_y
    is_moving_downwards = current_y < target_y

    # Define sensor angles and indices for each direction
    upward_sensors = [(270, 7), (180, 5), (0, 2), (90, 4), (90, 3)]
    downward_sensors = [(270, 4), (270, 3), (180, 2), (0, 5), (90, 7)]

    obstacle_in_front_upwards = is_moving_upwards and is_obstacle_in_direction(theta, upward_sensors, sonar_sensors)
    obstacle_in_front_downwards = is_moving_downwards and is_obstacle_in_direction(theta, downward_sensors, sonar_sensors)

    return obstacle_in_front_upwards or obstacle_in_front_downwards



def calculate_rotation_speed(robot_orientation, target_angle):
    if abs(robot_orientation - target_angle) < 1:
        return 1, 1  # Move forward if already aligned
    if robot_orientation < 90 or robot_orientation > 270:
        return -6, 6  # Rotate counterclockwise
    else:
        return 6, -6  # Rotate clockwise

def navigate_towards_goal(start_x, goal_x, start_y, goal_y, robot_orientation):
    if start_y < goal_y and not is_horizontal_aligned(start_y):
        if not(abs(robot_orientation - 90) < 1):
            if robot_orientation < 90 or robot_orientation > 270:
                return -6, 6
            else:
                return 6, -6
        else:
            return 1, 1
    
    elif start_y > goal_y and not is_horizontal_aligned(start_y):
        if not(abs(robot_orientation - 270) < 1):
            if robot_orientation < 90 or robot_orientation > 270:
                return 6, -6
            else:
                return -6, 6
        else:
            return 1, 1



def navigate_following_right_wall():
    if has_front_wall:
        print('Front wall detected')
        return -6, 6
    elif is_at_right_corner:
        print('Right corner detected')
        return -6, 6
    elif has_right_wall:
        print('Following right wall')
        return 1, 1
    else:
        print('No wall on the right')
        return 6, 0

def navigate_following_left_wall():
    if has_front_wall:
        print('Front wall detected')
        return 6, -6
    elif is_at_left_corner:
        print('Left corner detected')
        return 6, -6
    elif has_left_wall:
        print('Following left wall')
        return 1, 1
    else:
        print('No wall on the left')
        return 0, 6


# Main section
for time_step in range(NUM_TIMESTEPS):
    # Retrieve current orientation and position
    current_orientation = calculate_heading(compass_sensor.getValues())
    current_position = gps_sensor.getValues()

    # Detect walls
    has_right_wall = sonar_sensors[2].getValue() > 80
    has_left_wall = sonar_sensors[5].getValue() > 80
    has_front_wall = sonar_sensors[7].getValue() > 80
    is_at_right_corner = sonar_sensors[0].getValue() > 80 or sonar_sensors[1].getValue() > 80
    is_at_left_corner = sonar_sensors[6].getValue() > 80

    # Default speed
    motor_speeds = 0, 0
    TURN_RIGHT = False
    
    # Check if the goal is reached
    if calculate_distance(current_position[0], current_position[1], TARGET_X, TARGET_Y) > 0.1:
        if not OBSTACLE_DETECTED:
            current_direction = 'towards_goal'
            # Navigate towards goal until an obstacle is encountered
            motor_speeds = toward_goal()
            if has_front_wall or has_right_wall :
                print('Front wall detected')
                OBSTACLE_DETECTED = True
        else:
            if OBSTACLE_THRESHOLD < 0.3 :
                TURN_RIGHT = True
            else :
                if calc_teta() > 0:
                    OBSTACLE_DETECTED = False
                
            if not TURN_RIGHT :
               motor_speeds = navigate_following_right_wall()
            else :
               motor_speeds = navigate_following_left_wall() 
            # motor_speeds = navigate_following_left_wall()
            current_direction = 'wall_following'
            # Record minimum distance to goal
            distance_to_goal = calculate_distance(current_position[0], current_position[1], TARGET_X, TARGET_Y)
            if distance_to_goal <= OBSTACLE_THRESHOLD:
                OBSTACLE_THRESHOLD = distance_to_goal
                print('Minimum distance to goal:', OBSTACLE_THRESHOLD)
            

        previous_direction = current_direction
        left_motor.setVelocity(motor_speeds[0])
        right_motor.setVelocity(motor_speeds[1])
    else:
        print('Goal reached!')
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print('Time taken:', time_step)
        break

    # Proceed to the next simulation step
    robot.step()