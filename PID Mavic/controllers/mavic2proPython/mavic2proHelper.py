def getMotorAll(robot):
	frontLeftMotor = robot.getDevice('front left propeller')
	frontRightMotor = robot.getDevice('front right propeller')
	backLeftMotor = robot.getDevice('rear left propeller')
	backRightMotor = robot.getDevice('rear right propeller')
	return [frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor]

def motorsSpeed(robot, v1, v2, v3, v4):
	[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = getMotorAll(robot)
	frontLeftMotor.setVelocity(v1)
	frontRightMotor.setVelocity(v2)
	backLeftMotor.setVelocity(v3)
	backRightMotor.setVelocity(v4)
	return

def initialiseMotors(robot, MAX_PROPELLER_VELOCITY):
	[frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor] = getMotorAll(robot)
	frontLeftMotor.setPosition(float('inf'))
	frontRightMotor.setPosition(float('inf'))
	backLeftMotor.setPosition(float('inf'))
	backRightMotor.setPosition(float('inf'))

	motorsSpeed(robot, 0, 0, 0, 0)
	return