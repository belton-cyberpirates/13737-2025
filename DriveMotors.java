package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Collections;
import java.util.Arrays;
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


public class DriveMotors {

	enum states {
		ODOMETRY,
		DISTANCE,
		POWER,
		IDLE
	}

	static PIDController distanceSensorPidController = new PIDController(0, 0, 0);
	static PIDController xPosPidController = new PIDController(0.00255, 0.0000002, 0.000005);
	static PIDController yPosPidController = new PIDController(0.00255, 0.0000002, 0.0000045);
	static PIDController imuPidController = new PIDController(1, 0, 0.001);

	static Orientation angles;

	public DcMotorEx frontLeft;
	public DcMotorEx frontRight;
	public DcMotorEx backLeft;
	public DcMotorEx backRight;

	public GoBildaPinpointDriver odometry;

	private LinearOpMode auto;

	private DistanceSensor distSensor;

	ElapsedTime deltaTimer = new ElapsedTime();

	public states state = states.IDLE;

	public double targetX;
	public double targetY;
	public double targetHeading;

	public int targetDistance;
	
	ElapsedTime odometryTimer = new ElapsedTime();


	public DriveMotors(LinearOpMode auto) {
		this.auto = auto;

		this.frontRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_RIGHT_WHEEL_NAME);
		this.frontLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_LEFT_WHEEL_NAME);
		this.backLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_LEFT_WHEEL_NAME);
		this.backRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_RIGHT_WHEEL_NAME);
		
		this.backLeft.setTargetPosition(0);
		this.frontLeft.setTargetPosition(0);
		this.frontRight.setTargetPosition(0);
		this.backRight.setTargetPosition(0);
	
		//this.distSensor = auto.hardwareMap.get(DistanceSensor.class, BotConfig.DISTANCE_SENSOR_NAME);

		this.odometry = auto.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

		ResetEncoders();
		SetToRunWithPower();
		SetZeroBehaviour();
	}


	public void InitializeOdometry() {
		/*
		Set the odometry pod positions relative to the point that the odometry computer tracks around.
		The X pod offset refers to how far sideways from the tracking point the
		X (forward) odometry pod is. Left of the center is a positive number,
		right of center is a negative number. the Y pod offset refers to how far forwards from
		the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
		backwards is a negative number.
		 */
		this.odometry.setOffsets(145, -70); 

		/*
		Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
		the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
		If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
		number of ticks per mm of your odometry pod.
		 */
		this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
		//odo.setEncoderResolution(13.26291192);


		/*
		Set the direction that each of the two odometry pods count. The X (forward) pod should
		increase when you move the robot forward. And the Y (strafe) pod should increase when
		you move the robot to the left.
		 */
		this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


		/*
		Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
		The IMU will automatically calibrate when first powered on, but recalibrating before running
		the robot is a good idea to ensure that the calibration is "good".
		resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
		This is recommended before you run your autonomous, as a bad initial calibration can cause
		an incorrect starting value for x, y, and heading.
		 */
		//odo.recalibrateIMU();
		this.odometry.resetPosAndIMU();
	}


	public void process() {
		double deltaTime = deltaTimer.seconds();

		switch (this.state) {
			case ODOMETRY:
				driveWithOdometry(deltaTime);
				break;
			
			case DISTANCE:
				driveWithDistanceSensor(deltaTime);
				break;
		}
		
		this.odometry.update();
		deltaTimer.reset();
	}


	public void DriveWithPower(double backLeftPower, double frontLeftPower, double frontRightPower, double backRightPower) {
		this.state = states.POWER;

		backLeft.setPower(backLeftPower);
		frontLeft.setPower(frontLeftPower);
		frontRight.setPower(frontRightPower);
		backRight.setPower(backRightPower);
	}


	private void driveWithOdometry(double delta) {

		double heading = odometry.getHeading();

		double xDir = xPosPidController.PIDControl(targetX, odometry.getPosX(), delta);
		double yDir = yPosPidController.PIDControl(targetY, odometry.getPosY(), delta);
		double anglePower = imuPidController.PIDControlRadians(targetHeading, heading, delta);

		// Rotate the movement vector to convert field relative direction to robot relative direction
		double rotatedX =
			xDir * Math.cos(-heading) -
			yDir * Math.sin(-heading);
		double rotatedY =
			xDir * Math.sin(-heading) +
			yDir * Math.cos(-heading);

		// Strafing is slower than rolling, bump speed
		rotatedY *= BotConfig.STRAFE_MULT;

		// Set the power of the wheels based off the new movement vector
		double backLeftPower   = (-rotatedX - rotatedY + anglePower);
		double frontLeftPower  = (-rotatedX + rotatedY + anglePower);
		double frontRightPower = ( rotatedX + rotatedY + anglePower);
		double backRightPower  = ( rotatedX - rotatedY + anglePower);

		// Find highest motor power value
		double highestPower = Collections.max(Arrays.asList( Math.abs(backLeftPower), Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backRightPower) ));

		// Scale power values if trying to run motors faster than possible
		if (highestPower > 1) {
			backLeftPower /= highestPower;
			frontLeftPower /= highestPower;
			frontRightPower /= highestPower;
			backRightPower /= highestPower;
		}

		// // If trying to move at full power, scale down to 90%
		// if (highestPower > .9) {
		// 	backLeftPower *= .9;
		// 	frontLeftPower *= .9;
		// 	frontRightPower *= .9;
		// 	backRightPower *= .9;
		// }

		backLeft.setPower(backLeftPower);
		frontLeft.setPower(frontLeftPower);
		frontRight.setPower(frontRightPower);
		backRight.setPower(backRightPower);
		
		auto.telemetry.addData("drivemotors heading", heading);
		auto.telemetry.addData("drivemotors anglePower", anglePower);
		
		auto.telemetry.addData("drivemotors xDir", xDir);
		auto.telemetry.addData("drivemotors yDir", yDir);
		auto.telemetry.addData("drivemotors anglePower", anglePower);
	}


	public void driveWithDistanceSensor(double delta) {
		double dist = distSensor.getDistance(DistanceUnit.MM);
		double error = targetDistance - dist;
		
		double power = distanceSensorPidController.PIDControl(error, delta);

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
		this.targetHeading = -( heading * ( Math.PI / 180 ) );

		this.odometryTimer.reset();

		this.state = states.ODOMETRY;
	}


	public void MoveToDistance(int distance) {
		this.targetDistance = distance;

		this.state = states.DISTANCE;
	}


	public void Stop() {
		this.state = states.IDLE;
	}


	public boolean isDone() {
		switch (this.state) {
			case ODOMETRY:
				return odometryTimer.milliseconds() > 750 && 
					(Math.abs(xPosPidController.lastError) < 17) && // max vertical error - MM
					(Math.abs(yPosPidController.lastError) < 17) && // max horizontal error - MM
					(Math.abs(imuPidController.lastError) < .03); // max angle error - radians
			
			case DISTANCE:
				return (Math.abs(distanceSensorPidController.lastError) < 5);
		}
		
		return true;
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
	
	
	private void ResetEncoders() {
		this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}
}
