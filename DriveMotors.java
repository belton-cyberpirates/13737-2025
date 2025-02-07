package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.PIDController;


enum states {
	ODOMETRY,
	DISTANCE,
	IDLE
}


public class DriveMotors {

	static PIDController distanceSensorPidController = new PIDController(0.007, 0.0005, 0.00018);
	static PIDController xPosPidController = new PIDController(0, 0, 0);
	static PIDController yPosPidController = new PIDController(0, 0, 0);
	static PIDController imuPidController = new PIDController(0.01, 0, 0);

	static Orientation angles;

	public DcMotorEx frontLeft;
	public DcMotorEx frontRight;
	public DcMotorEx backLeft;
	public DcMotorEx backRight;

	public Odometry odometry;

	private LinearOpMode auto;

	private DistanceSensor distSensor;

	ElapsedTime deltaTimer = new ElapsedTime();

	states state = states.IDLE;

	double targetX;
	double targetY;
	double targetHeading;

	int targetDistance;


	public DriveMotors(LinearOpMode auto) {
		this.auto = auto;

		this.frontRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_RIGHT_WHEEL_NAME);
		this.frontLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_LEFT_WHEEL_NAME);
		this.backLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_LEFT_WHEEL_NAME);
		this.backRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_RIGHT_WHEEL_NAME);
	
		this.distSensor = auto.hardwareMap.get(DistanceSensor.class, BotConfig.DISTANCE_SENSOR_NAME);

		this.odometry = new Odometry(auto);

		SetToRunWithPower();
		SetZeroBehaviour();
	}


	public process() {
		double deltaTime = deltaTimer.seconds();

		switch this.state {
			case ODOMETRY:
				driveWithOdometry();
				break;
			
			case DISTANCE:
				driveWithDistanceSensor();
				break;
		}
		
		auto.telemetry.update();
		deltaTimer.reset();
	}


	private void driveWithOdometry() {

		double heading = auto.getHeading();

		double xDir = xPosPidController.PIDControl(targetX, odometry.getX(), deltaTime);
		double yDir = yPosPidController.PIDControl(targetY, odometry.getY(), deltaTime);
		double anglePower = imuPidController.PIDControlRadians(targetAngle, heading, deltaTimer.seconds());

		// Rotate the movement vector to cancel out the angle of the robot
		double rotatedX =
			xDir * Math.cos(botHeading) -
			yDir * Math.sin(botHeading);
		double rotatedY =
			xDir * Math.sin(botHeading) +
			yDir * Math.cos(botHeading);

		// Strafing is slower than rolling, bump speed
		rotatedX *= BotConfig.STRAFE_MULT;

		// Set the power of the wheels based off the new movement vector
		double backLeftPower   = (-rotatedY - rotatedX + anglePower);
		double frontLeftPower  = (-rotatedY + rotatedX + anglePower);
		double frontRightPower = ( rotatedY + rotatedX + anglePower);
		double backRightPower  = ( rotatedY - rotatedX + anglePower);

		// Find highest motor power value
		double highestPower =  Collections.max(Arrays.asList( backLeftPower, frontLeftPower, frontRightPower, backRightPower ));

		// Scale power values if trying to run motors faster than possible
		if (highestPower > 1) {
			backLeftPower /= highestPower;
			frontLeftPower /= highestPower;
			frontRightPower /= highestPower;
			backRightPower /= highestPower;
		}

		backLeft.setPower(backLeftPower);
		frontLeft.setPower(frontLeftPower);
		frontRight.setPower(frontRightPower);
		backRight.setPower(backRightPower);
	}


	public void driveWithDistanceSensor() {
		double dist = distSensor.getDistance(DistanceUnit.MM);
		double error = targetDistance - dist;
		
		double power = distanceSensorPidController.PIDControl(error, deltaTime);

		frontLeft.setPower(-power);
		frontRight.setPower(power);
		backLeft.setPower(-power);
		backRight.setPower(power);
		
		auto.telemetry.addData("dist", dist);
		auto.telemetry.addData("error", error);
		auto.telemetry.addData("power", power);
	}


	public void Move(double xPos, double yPos, double heading) {
		this.targetX = xPos;
		this.targetY = yPos;
		this.targetHeading = heading;

		this.state = states.ODOMETRY;
	}


	public void MoveToDistance(int distance) {
		this.targetDistance = distance;

		this.state = states.DISTANCE;
	}


	public void Stop() {
		this.state = states.IDLE;
	}


  	/*public void TurnToAngle(double targetAngleDegrees int time) {
		SetToRunWithPower();
		
		ElapsedTime deltaTimer = new ElapsedTime();
		ElapsedTime timer = new ElapsedTime();
		
		double error = Double.POSITIVE_INFINITY;
		
		while (auto.opModeIsActive()) {
			double heading = auto.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
			
			double power = imuPidController.PIDControlRadians(targetAngle, heading, deltaTimer.seconds());
			deltaTimer.reset();
			frontLeft.setPower(power);
			frontRight.setPower(power);
			backLeft.setPower(power);
			backRight.setPower(power);
			
			auto.telemetry.addData("heading", heading);
			auto.telemetry.addData("target", targetAngle)
			auto.telemetry.addData("error", error);
			auto.telemetry.addData("power", power);
			auto.telemetry.update();
		}
	}*/


	public boolean isDone() {
		switch this.state {
			case ODOMETRY:
				return (Math.abs(xPosPidController.error) < 5) && (Math.abs(yPosPidController.error) < 5) && (Math.abs(imuPidController.error) < .1);
				break;
			
			case DISTANCE:
				return (Math.abs(distanceSensorPidController.error) < 5);
				break;
	}


	private void SetToRunWithPower() {
		this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
  
  
	private void SetZeroBehaviour() {
		this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}
}
