
#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration
FrontRightDrive = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)

FrontLeftDrive = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)

BackRightDrive = Motor(Ports.PORT10, GearSetting.RATIO_6_1, False)

BackLeftDrive = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)

Intake = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)

Conveyor = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)

Clamp = DigitalOut(brain.three_wire_port.h)

controller_1 = Controller(PRIMARY)

myVariable = 0

def when_started1():
    global myVariable
    pass

def onauton_autonomous_0():
    moveBackward2(30)
    Clamp.set(True)
    Conveyor.spin(FORWARD)
    Conveyor.set_velocity(-100, PERCENT)
    wait(2, SECONDS)
    Conveyor.set_velocity(0, PERCENT)
    moveBackward2(20)
    moveForward2(10)
    motorTooHotScream(50)
   
    global myVariable
    pass

def ondriver_drivercontrol_0():
    FrontRightDrive.set_stopping(COAST)
    BackRightDrive.set_stopping(COAST)
    FrontLeftDrive.set_stopping(COAST)
    BackLeftDrive.set_stopping(COAST)

    BackRightDrive.spin(FORWARD);

    BackLeftDrive.spin(FORWARD);

    FrontRightDrive.spin(FORWARD);

    FrontLeftDrive.spin(FORWARD);

    Intake.spin(FORWARD);
    
    Conveyor.spin(FORWARD);

    timer.clear()
    while (True):
        posR = (BackRightDrive.position(TURNS) + FrontRightDrive.position(TURNS))/2
        posL = (BackLeftDrive.position(TURNS) + FrontLeftDrive.position(TURNS))/2

        

        if controller_1.buttonA.pressing():
            Clamp.set(True)
        if controller_1.buttonB.pressing() :
            Clamp.set(False)


        rExtra = 0
        lExtra = 0
        # if L is slower than R, add rpm version of position to Left velocity
        if (posR - posL < 0):
            # 1 rps = 60 rpm, 1 percent = 6 rpm, 60/6 = 10
            lExtra = (posR-posL)/timer.value() * 10
        # vice versa
        else:
            rExtra = (posL-posR)/timer.value() * 10
        
        rVelocity = (controller_1.axis1.position()*(-1) - controller_1.axis3.position()) * (0.7) * rMod + rExtra
        lVelocity = (controller_1.axis1.position() - controller_1.axis3.position()) * -0.7 * lMod + lExtra
        BackLeftDrive.set_velocity(lVelocity, PERCENT)

        FrontLeftDrive.set_velocity(lVelocity, PERCENT)

        BackRightDrive.set_velocity(rVelocity, PERCENT)

        FrontRightDrive.set_velocity(rVelocity, PERCENT)

        Intake.set_velocity((controller_1.buttonL1.pressing()-controller_1.buttonR1.pressing()) * 100, PERCENT)

        Conveyor.set_velocity((controller_1.buttonL2.pressing()-controller_1.buttonR2.pressing()) * 100, PERCENT)

        timer.clear()
        resetPositions()
        motorTooHotScream(50)
        

# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # Start the autonomous control tasks
    auton_task_0 = Thread( onauton_autonomous_0 )
    # wait for the driver control period to end
    while( competition.is_autonomous() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the autonomous control tasks
    auton_task_0.stop()

def vexcode_driver_function():
    # Start the driver control tasks
    driver_control_task_0 = Thread( ondriver_drivercontrol_0 )

    # wait for the driver control period to end
    while( competition.is_driver_control() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the driver control tasks
    driver_control_task_0.stop()



def moveBackward(inches):
    resetPositions()

    FrontRightDrive.spin(FORWARD)
    BackRightDrive.spin(FORWARD)
    FrontLeftDrive.spin(FORWARD)
    BackLeftDrive.spin(FORWARD)

    curPosTotal = 0
    wheelCirc = 3.25
    pi = 3.1415
    gearRatio = 3/5
    turnDist = wheelCirc * pi * gearRatio
    speed = 100
    minModifier = 0.1
    modifier = 1
    tooFarDistInches = 0.1

    rightMod = 1
    leftMod = 1

    while curPosTotal * turnDist < inches or curPosTotal * turnDist > inches+tooFarDistInches:
        
        curPosTotal = (-(FrontRightDrive.position(TURNS) + BackRightDrive.position(TURNS)) + (FrontLeftDrive.position(TURNS) + BackLeftDrive.position(TURNS)))/4

        # 100% speed minus the percent distance the robot has gone
        if (curPosTotal * turnDist) / inches > minModifier or (curPosTotal * turnDist) / inches < -minModifier:
            modifier = (1 - ((curPosTotal * turnDist) / inches))
        elif (curPosTotal * turnDist) / inches) < 0:
            modifier = -minModifier
        else:
            modifier = minModifier

        # put together all the modifiers and add 1 so it at least WILL eventually get there. Had an issue with this before.
        FrontRightDrive.set_velocity(speed * rightMod * modifier + 1, PERCENT)
        BackRightDrive.set_velocity(speed * rightMod * modifier + 1, PERCENT)
        FrontLeftDrive.set_velocity((speed * leftMod * modifier + 1) * -1, PERCENT)
        BackLeftDrive.set_velocity((speed * leftMod * modifier + 1) * -1, PERCENT)
    FrontRightDrive.set_velocity(0, PERCENT)
    BackRightDrive.set_velocity(0, PERCENT)
    FrontLeftDrive.set_velocity(0, PERCENT)
    BackLeftDrive.set_velocity(0, PERCENT)

def moveForward(inches):
    resetPositions()

    FrontRightDrive.spin(FORWARD)
    BackRightDrive.spin(FORWARD)
    FrontLeftDrive.spin(FORWARD)
    BackLeftDrive.spin(FORWARD)

    curPosTotal = 0
    wheelCirc = 3.25
    pi = 3.1415
    gearRatio = 3/5
    turnDist = wheelCirc * pi * gearRatio
    speed = 100
    minModifier = 0.1
    modifier = 1
    tooFarDistInches = 0.1

    rightMod = 1
    leftMod = 1

    while curPosTotal * turnDist < inches or curPosTotal * turnDist > inches+tooFarDistInches:
        
        curPosTotal = (-(FrontRightDrive.position(TURNS) + BackRightDrive.position(TURNS)) + (FrontLeftDrive.position(TURNS) + BackLeftDrive.position(TURNS)))/4

        # 100% speed minus the percent distance the robot has gone
        if (curPosTotal * turnDist) / inches > minModifier or (curPosTotal * turnDist) / inches < -minModifier:
            modifier = (1 - ((curPosTotal * turnDist) / inches))
        elif (curPosTotal * turnDist) / inches) < 0:
            modifier = -minModifier
        else:
            modifier = minModifier
        
        # put together all the modifiers and add 1 so it at least WILL eventually get there. Had an issue with this before.
        FrontRightDrive.set_velocity((speed * rightMod * modifier + 1) * -1, PERCENT)
        BackRightDrive.set_velocity((speed * rightMod * modifier + 1) * -1, PERCENT)
        FrontLeftDrive.set_velocity((speed * leftMod * modifier + 1), PERCENT)
        BackLeftDrive.set_velocity((speed * leftMod * modifier + 1), PERCENT)
    FrontRightDrive.set_velocity(0, PERCENT)
    BackRightDrive.set_velocity(0, PERCENT)
    FrontLeftDrive.set_velocity(0, PERCENT)
    BackLeftDrive.set_velocity(0, PERCENT)

def turnLeft(turns):
    fullTurnTurns = 360/55
    BackRightDrive.spin_for(FORWARD, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)
    BackLeftDrive.spin_for(FORWARD, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)
    FrontRightDrive.spin_for(FORWARD, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)
    FrontLeftDrive.spin_for(FORWARD, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)

def turnRight(turns):
    fullTurnTurns = 360/55
    BackRightDrive.spin_for(REVERSE, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)
    BackLeftDrive.spin_for(REVERSE, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)
    FrontRightDrive.spin_for(REVERSE, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)
    FrontLeftDrive.spin_for(REVERSE, turns * fullTurnTurns, TURNS, 50, PERCENT, wait=False)

def moveForward2(inches):
    wheelCirc = 3.25
    pi = 3.1415
    gearRatio = 3/5
    turnDist = wheelCirc * pi * gearRatio
    speed = 40

    BackRightDrive.spin_for(FORWARD, inches/turnDist, TURNS, speed, PERCENT, wait=False)
    BackLeftDrive.spin_for(REVERSE, inches/turnDist, TURNS, speed, PERCENT, wait=False)
    FrontRightDrive.spin_for(FORWARD, inches/turnDist, TURNS, speed, PERCENT, wait=False)
    FrontLeftDrive.spin_for(REVERSE, inches/turnDist, TURNS, speed, PERCENT, wait=False)

def moveBackward2(inches):
    wheelCirc = 3.25
    pi = 3.1415
    gearRatio = 3/5
    turnDist = wheelCirc * pi * gearRatio
    speed = 40

    BackRightDrive.spin_for(REVERSE, inches/turnDist, TURNS, speed, PERCENT, wait=False)
    BackLeftDrive.spin_for(FORWARD, inches/turnDist, TURNS, speed, PERCENT, wait=False)
    FrontRightDrive.spin_for(REVERSE, inches/turnDist, TURNS, speed, PERCENT, wait=False)
    FrontLeftDrive.spin_for(FORWARD, inches/turnDist, TURNS, speed, PERCENT, wait=False)
    
def moveAndTurn(circleRadius, percentOfCircle, movingRight):
    wheelCirc = 3.25
    pi = 3.1415
    gearRatio = 3/5
    turnDist = wheelCirc * pi * gearRatio

    turnCircleCircumference = circleRadius * pi * 2

    resetPositions()

    FrontRightDrive.spin(FORWARD)
    BackRightDrive.spin(FORWARD)
    FrontLeftDrive.spin(FORWARD)
    BackLeftDrive.spin(FORWARD)

    speed = 50

    if (movingRight):
        curPosTotal = 0
        while (curPosTotal * turnDist < percentOfCircle/turnCircleCircumference):
            #curPosTotal*turnDist = arc length
            curPosTotal = (-(FrontRightDrive.position(TURNS) + BackRightDrive.position(TURNS)) + (FrontLeftDrive.position(TURNS) + BackLeftDrive.position(TURNS)))/4
            # in radians. L*2pi / 2pi*r
            centralAngle = ((curPosTotal*turnDist)*(2*pi))/turnCircleCircumference

            BackRightDrive.set_velocity(math.cos(centralAngle) * speed, PERCENT)
            FrontRightDrive.set_velocity(math.cos(centralAngle) * speed, PERCENT)
            BackLeftDrive.set_velocity(speed, PERCENT)
            FrontLeftDrive.set_velocity(speed, PERCENT)
    else:
        curPosTotal = 0
        while (curPosTotal * turnDist < percentOfCircle/turnCircleCircumference):
            #curPosTotal*turnDist = arc length
            curPosTotal = (-(FrontRightDrive.position(TURNS) + BackRightDrive.position(TURNS)) + (FrontLeftDrive.position(TURNS) + BackLeftDrive.position(TURNS)))/4
            # in radians. L*2pi / 2pi*r
            centralAngle = ((curPosTotal*turnDist)*(2*pi))/turnCircleCircumference

            BackRightDrive.set_velocity(speed, PERCENT)
            FrontRightDrive.set_velocity(speed, PERCENT)
            BackLeftDrive.set_velocity(math.cos(centralAngle) * speed, PERCENT)
            FrontLeftDrive.set_velocity(math.cos(centralAngle) * speed, PERCENT)

def moveAndTurn2(circleRadius, circleTurns, movingRight):
    wheelCirc = 3.25
    pi = 3.1415
    gearRatio = 3/5
    turnDist = wheelCirc * pi * gearRatio
    quarterTurnTurns = (360/55) * 0.25

    turnCircleCircumference = circleRadius * pi * 2

    resetPositions()

    FrontRightDrive.spin(FORWARD)
    BackRightDrive.spin(FORWARD)
    FrontLeftDrive.spin(FORWARD)
    BackLeftDrive.spin(FORWARD)

    speed = 50

    # FIGURE OUT HOW MANY DEGREES ONE TURN OF A SIDE DOES WITH BELOW FUNCTION

def sideTurn():
    FrontRightDrive.spin_for(FORWARD, 1, TURNS, 100, PERCENT, wait=False)
    BackRightDrive.spin_for(FORWARD, 1, TURNS, 100, PERCENT, wait=False)

def resetPositions():
    FrontRightDrive.set_position(0,TURNS)
    BackRightDrive.set_position(0,TURNS)
    FrontLeftDrive.set_position(0,TURNS)
    BackLeftDrive.set_position(0,TURNS)

def motorTooHotScream(maxTemp){
    motors = [FrontRightDrive, FrontLeftDrive, BackRightDrive, BackLeftDrive, Intake, Conveyor]
    for i in range(len(motors)):
        while (motors[i].temperature(PERCENT) > maxTemp):
            brain.screen.println("A motor is too hot!")
}

# register the competition functions
competition = Competition( vexcode_driver_function, vexcode_auton_function )

when_started1()
