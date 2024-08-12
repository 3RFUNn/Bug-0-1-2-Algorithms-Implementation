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
compass_sensor = robot.getDevice("compass")
compass_sensor.enable(1)

sonar5 = robot.getDevice('ps5')
sonar5.enable(1)
sonar6 = robot.getDevice('ps6')
sonar6.enable(1)
sonar7 = robot.getDevice('ps7')
sonar7.enable(1)

robot.step(1000)
initial_gps_reading = gps_sensor.getValues()
initial_compass_reading = compass_sensor.getValues()

first_move = False
max_speed = 6.28

left_motor.setVelocity(1)
right_motor.setVelocity(1)

with open('map_data.txt', 'w') as file:
    for time_step in range(7040):#7040
        left_speed = 0
        right_speed = 0
        
        location = gps_sensor.getValues()

        if not first_move: # first move
            left_speed,right_speed = max_speed,max_speed
            if sonar7.getValue() > 80:
                first_move =True
        elif math.sqrt((0-location[0])*(0-location[0]) + (-0.5-location[1])*(-0.5-location[1])) < 0.05: # goal
            print('End')
            break

        else: # follow the wall
            if sonar7.getValue() > 80:
                # print('Front wall')
                left_speed = max_speed
                right_speed = -max_speed
            elif sonar6.getValue() > 80:
                # print('Left corner')
                left_speed = max_speed
                right_speed = -max_speed
            elif sonar5.getValue() > 80:
                # print('Wall following')
                left_speed = max_speed
                right_speed = max_speed
            else:
                # print('Right corner')
                left_speed = 0
                right_speed = max_speed

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        robot.step()

left_motor.setVelocity(0)
right_motor.setVelocity(0)
