"""my_controller controller."""

from numpy import inf
from sympy import *

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Lidar

MAX_SPEED = 6
BASE_SPEED = 2

# create the Robot instance.
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
sensors = []
speed = [0,0]

weights = [ [-0.3, -0.4], [-0.3,-0.4], [-0.5, 0.5], [0.1, 0.1], [0.5, -0.5]]

sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 FrontLeft"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 FrontRight"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 Right"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 Back"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 Left"))

for i in sensors:
    i.enable(TIME_STEP)


rightMotor = robot.getDevice('right wheel motor')
leftMotor = robot.getDevice('left wheel motor')


def enableSpeedControl(motor, initVelocity = 0.):
    motor.setPosition(float('inf'))
    motor.setVelocity(initVelocity)

def getSensorData():
    def voltageToMetersFormula():
        x = symbols('x')
        return 1.784*(x**(-0.4215)) - 1.11

    sensorDataMeters = []
    for j in sensors:
        sensorDataMeters.append(voltageToMetersFormula().evalf(subs={x:j.getValue()}))

    return sensorDataMeters

def braitenberg():
    i = 0
    j = 0
    while (i < 2):
        speed[i] = 0.0
        while (j < 5):
            if (sensorDataMeters[j] > 1.5):
                j += 1
                continue
            speed[i] += sensorDataMeters[j] * weights[j][i]
            j += 1

        speed[i] = 4.0 + speed[i] * MAX_SPEED
        if (speed[i] > MAX_SPEED):
            speed[i] = MAX_SPEED
        elif (speed[i] < -MAX_SPEED):
            speed[i] = -MAX_SPEED;

        i += 1

def setActuators():
    leftMotor.setVelocity(float(speed[0]))
    rightMotor.setVelocity(float(speed[1]))

    
enableSpeedControl(leftMotor, 0.0)
enableSpeedControl(rightMotor, 0.0)

x = symbols('x')

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    
    

    enableSpeedControl(leftMotor, 0.0)
    enableSpeedControl(rightMotor, 0.0)

    sensorDataMeters = getSensorData()
    print("FrontLeft: " + str(sensorDataMeters[0]))
    print("FrontRight: " + str(sensorDataMeters[1]))
    print("Right: " + str(sensorDataMeters[2]))
    print("Left: " + str(sensorDataMeters[4]))
    print("Back: " + str(sensorDataMeters[3]))

    braitenberg()
    setActuators()

    #print(sensors[3].getValue())
    #print(sensorDataMeters[3])