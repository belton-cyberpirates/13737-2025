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
		IDLE
	}

	static PIDController distanceSensorPidController = new PIDController(0.007, 0.0005, 0.00018);
	static PIDController xPosPidController = new PIDController(0.015, 0.00002, 0.0005);
	static PIDController yPosPidController = new PIDController(0.015, 0.00002, 0.0005);
	static PIDController imuPidController = new PIDController(1.8, 0, 0.033);

	static Orientation angles;

	public DcMotorEx frontLeft;
	public DcMotorEx frontRight;
	public DcMotorEx backLeft;
	public DcMotorEx backRight;

	public Odometry odometry;

	private Auto auto;

	private DistanceSensor distSensor;

	ElapsedTime deltaTimer = new ElapsedTime();

	public states state = states.IDLE;

	public double targetX;
	public double targetY;
	public double targetHeading;

	public int targetDistance;
	
	ElapsedTime odometryTimer = new ElapsedTime();


	public DriveMotors(Auto auto) {
		this.auto = auto;

		this.frontRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_RIGHT_WHEEL_NAME);
		this.frontLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.FRONT_LEFT_WHEEL_NAME);
		this.backLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_LEFT_WHEEL_NAME);
		this.backRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.BACK_RIGHT_WHEEL_NAME);
	
		this.distSensor = auto.hardwareMap.get(DistanceSensor.class, BotConfig.DISTANCE_SENSOR_NAME);

		this.odometry = new Odometry(auto);

		ResetEncoders();
		SetToRunWithPower();
		SetZeroBehaviour();
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
		
		auto.telemetry.update();
		deltaTimer.reset();
	}


	private void driveWithOdometry(double delta) {

		double heading = auto.getHeading();

		double xDir = xPosPidController.PIDControl(targetX, odometry.getX(), delta);
		double yDir = yPosPidController.PIDControl(targetY, odometry.getY(), delta);
		double anglePower = imuPidController.PIDControlRadians(targetHeading, heading, delta);

		// Rotate the movement vector to cancel out the angle of the robot
		double rotatedX =
			xDir * Math.cos(heading) -
			yDir * Math.sin(heading);
		double rotatedY =
			xDir * Math.sin(heading) +
			yDir * Math.cos(heading);

		// Strafing is slower than rolling, bump speed
		rotatedX *= BotConfig.STRAFE_MULT;

		// Set the power of the wheels based off the new movement vector
		double backLeftPower   = ( rotatedY - rotatedX + anglePower);
		double frontLeftPower  = ( rotatedY + rotatedX + anglePower);
		double frontRightPower = (-rotatedY + rotatedX + anglePower);
		double backRightPower  = (-rotatedY - rotatedX + anglePower);

		// Find highest motor power value
		double highestPower = Collections.max(Arrays.asList( Math.abs(backLeftPower), Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backRightPower) ));

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
		this.targetHeading = (heading * ( Math.PI / 180 ));

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
		switch (this.state) {
			case ODOMETRY:
				return odometryTimer.milliseconds() > 1000 &&
					(Math.abs(xPosPidController.lastOutput) < .05) && 
			  	(Math.abs(yPosPidController.lastOutput) < .05) && 
			  	(Math.abs(imuPidController.lastOutput) < .05);
			
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
