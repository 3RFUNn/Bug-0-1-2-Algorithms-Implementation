import sys
import math
from controller import Robot

webots_path = 'D:\\Apps\\Webots\\lib\\controller\\python'
sys.path.append(webots_path)

# Initialize the e-puck robot 
robot = Robot()

# Initialize  motors & sensors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0)
right_motor.setVelocity(0)

gps_sensor = robot.getDevice("gps")
gps_sensor.enable(1)

compass = robot.getDevice("compass")
compass.enable(1)

sonar5 = robot.getDevice('ps5')
sonar5.enable(1)
sonar6 = robot.getDevice('ps6')
sonar6.enable(1)
sonar7 = robot.getDevice('ps7')
sonar7.enable(1)

robot.step(1000)
initial_gps_reading = gps_sensor.getValues()

first_move = False
max_speed = 6.28



left_motor.setVelocity(1)
right_motor.setVelocity(1)

def calc_dist(start_x, start_y, goal_x, goal_y):
    return math.sqrt((start_x - goal_x)**2 + (start_y - goal_y)**2)

def toward_goal():

    for time_step in range(10000):
        pos = gps_sensor.getValues()

        north = compass.getValues()

        # Calculate robot's current angle (assuming north is along Y-axis)
        robot_angle = math.atan2(north[0], north[1])

        # Calculate angle to target
        target_angle = calculate_angle_to_target(pos[0], pos[2], target_x, target_y)

        # Calculate the angle difference
        angle_difference = get_angle_difference(robot_angle, target_angle)

        # Turn towards the target
        if abs(angle_difference) > 0.1:  # Threshold to avoid oscillations
            turn_speed = 1.0 if angle_difference > 0 else -1.0
            left_motor.setVelocity(-turn_speed)
            right_motor.setVelocity(turn_speed)
        else:
            # Move forward
            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(max_speed)

            # if abs(pos[0] - target_x) < 0.1 and abs(pos[2] - target_y) < 0.1:
            if calc_dist(target_x,target_y,location[0],location[1]) <0.1:
                print("Target reached!")
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                return True
            if sonar7.getValue() > 80:
                return False

        robot.step()

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

def wall_follow():
    if sonar7.getValue() > 80:
        left_speed = max_speed
        right_speed = -max_speed
    elif sonar6.getValue() > 80:
        left_speed = max_speed
        right_speed = -max_speed
    elif sonar5.getValue() > 80:
        left_speed = max_speed
        right_speed = max_speed
    else:
        left_speed = 0
        right_speed = max_speed

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)


min_dis = float('inf')
min_x = 0
min_y = 0
target_x, target_y = 0, -0.5
visited_locations = set()
state = 0 # 0:toward the goal or key_point , 1:wall following , 2:go back to key point

for time_step in range(1000000):#7040

    location = gps_sensor.getValues()
    if state == 0:
        print('Toward the line:')
        if toward_goal():
            break
        state = 1
        print('Wall folloing')
        x_start, y_start = location[0], location[1]
        
    elif state == 1:
        wall_follow()
        if time_step > 100 :
            if time_step% 5 == 0:
                dist = calc_dist(target_x,target_y,location[0],location[1])
                if min_dis > dist:
                    print('--key point: x:',min_x,' y:',min_y,' min distance:',min_dis)
                    min_dis = dist
                    min_x = location[0]
                    min_y = location[1]

            if round(x_start,1) == round(location[0], 1) and round(y_start,1) == round(location[1], 1):
                print('key point: x:',min_x,' y:',min_y,' min distance:',min_dis)
                print('back to minimum point')
                state = 2
                
    else:
        wall_follow()
        if round(min_x,1) == round(location[0], 1) and round(min_y,1) == round(location[1], 1):
            print('*key point: x:',min_x,' y:',min_y,' min distance:',min_dis)
            print('Toward the line')
            state = 0

    robot.step()


print('x:',location[0],' y:',location[1] )
left_motor.setVelocity(0)
right_motor.setVelocity(0)
