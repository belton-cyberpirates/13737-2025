package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Centric (main)", group="DriveCodes")
public class DriveCode extends LinearOpMode {
	
	// Drive constants
	final double BASE_SPEED = .5;
	final double MAX_BOOST = 0.6; // boost maxes out at an additional 60% of the base speed
	final double STRAFE_MULT = 1.2;

	// Arm constants
	final double ARM_VELOCITY = 750;
	final double WRIST_VELOCITY = 1000;
	final double WRIST_LOWER_MULT = .7;
	final double WRIST_BUTTON_MULT = 1.75;
	final int WRIST_GRAB_POS = 1030;
	
	// Winch Constants
	final double WINCH_POWER = 1;
	
	// Claw constants
	final double CLAW_LEFT_OPEN_POS = 0.2;
	final double CLAW_LEFT_CLOSE_POS = 0.07;
	final double CLAW_RIGHT_OPEN_POS = 0.8;
	final double CLAW_RIGHT_CLOSE_POS = 0.93;
	final double CLAW_LEFT_HALF_CLOSE_POS = (CLAW_LEFT_OPEN_POS + CLAW_LEFT_CLOSE_POS) / 2;
	final double CLAW_RIGHT_HALF_CLOSE_POS = (CLAW_RIGHT_OPEN_POS + CLAW_RIGHT_CLOSE_POS) / 2;
	final double CLAW_LEFT_FULL_OPEN_POS = 0.4;
	final double CLAW_RIGHT_FULL_OPEN_POS = 0.6;
	
	// Drive motors
	private DcMotorEx BackLeft;
	private DcMotorEx FrontLeft;
	private DcMotorEx FrontRight;
	private DcMotorEx BackRight;

	// Arm motors
	private DcMotorEx ArmLeft;
	private DcMotorEx ArmRight;
	private DcMotorEx Wrist;
	
	private DcMotorEx Winch;
	
	// Servos
	private Servo ClawLeft;
	private Servo ClawRight;
	
	// Sensors
	private IMU imu;
	private DistanceSensor DistSensor;

	// Other Classes
	private Odometry odometry;
	
	// Other variables
	private boolean slideFrozen;

	@Override
	public void runOpMode() throws InterruptedException {
		// Assign drive motors
		BackLeft = hardwareMap.get(DcMotorEx.class, "back_left");
		FrontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
		FrontRight = hardwareMap.get(DcMotorEx.class, "front_right");
		BackRight = hardwareMap.get(DcMotorEx.class, "back_right");

		// Assign arm motors
		ArmLeft = hardwareMap.get(DcMotorEx.class, "left_arm");
		ArmRight = hardwareMap.get(DcMotorEx.class, "right_arm");
		Wrist = hardwareMap.get(DcMotorEx.class, "wrist");
		
		// Assign winch motor
		Winch = hardwareMap.get(DcMotorEx.class, "winch");
		
		// Assign servos
		ClawLeft = hardwareMap.get(Servo.class, "claw_left");
		ClawRight = hardwareMap.get(Servo.class, "claw_right");
		
		imu = hardwareMap.get(IMU.class, "imu");
		
		DistSensor = hardwareMap.get(DistanceSensor.class, "dist_sensor");

		odometry = new Odometry(this);

		
		// Set zero power behaviours
		BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		ArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		ArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		Winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		/*ArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		ArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		Wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
		
		Winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		
		ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		Wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		Winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Wait for the start button to be pressed
		waitForStart();

		// Reset robot heading on startup (not initialization)
		imu.resetYaw();
		double savedHeading = getSavedHeading();
		boolean prevSpecHotkey = false;
		boolean prevBarHotkey = false;
		
		boolean hanging = false;
		
		double botHeading = 0;

		while (opModeIsActive()) {
			// Reset yaw when start button is pressed so that a restart is not needed if the yaw should be reset again.
			if (gamepad1.start) {
				imu.resetYaw();
			}

			
			// Gamepad variables
			double leftStickXGP1 = gamepad1.left_stick_x;
			double leftStickYGP1 = gamepad1.left_stick_y;
			double rightStickXGP1 = gamepad1.right_stick_x;
			double rightStickYGP1 = gamepad1.right_stick_y;
			
			double leftStickYGP2 = gamepad2.left_stick_y;
			double rightStickYGP2 = gamepad2.right_stick_y;


			// Get the speed the bot would go with the joystick pushed all the way
			double maxSpeed = calcMaxSpeed(gamepad1.right_trigger - gamepad1.left_trigger, BASE_SPEED, MAX_BOOST);

			// Get the heading of the bot (the angle it is facing) in radians
			double newHeading = (savedHeading + imu .getRobotYawPitchRollAngles() .getYaw(AngleUnit.RADIANS));
			//if (newHeading != 0) {
				botHeading = newHeading;
			//}


			// Virtually rotate the joystick by the negative angle of the robot
			double rotatedX =
				leftStickXGP1 * Math.cos(botHeading) -
				leftStickYGP1 * Math.sin(botHeading);
			double rotatedY =
				leftStickXGP1 * Math.sin(botHeading) +
				leftStickYGP1 * Math.cos(botHeading);
			
			// strafing is slower than rolling, bump speed
			rotatedX *= STRAFE_MULT;


			// Set the power of the wheels based off the new joystick coordinates
			// y+x+stick <- [-1,1]
			BackLeft.setPower(
				(-rotatedY - rotatedX + rightStickXGP1) * maxSpeed
			);
			FrontLeft.setPower(
				(-rotatedY + rotatedX + rightStickXGP1) * maxSpeed
			);
			FrontRight.setPower(
				(rotatedY + rotatedX + rightStickXGP1) * maxSpeed
			);
			BackRight.setPower(
				(rotatedY - rotatedX + rightStickXGP1) * maxSpeed
			);
			
			if (!gamepad2.right_bumper) {
				SetArmVelocity(gamepad2.left_stick_y * ARM_VELOCITY);
			}
			
			
			if (gamepad1.dpad_up) {
				hanging = true;
			}
			
			if (gamepad1.dpad_down) {
				Winch.setPower(-WINCH_POWER);
				hanging = false;
			}
			else if (hanging) {
				Winch.setPower(WINCH_POWER);
				ArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				ArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				ArmLeft.setPower(1);
				ArmRight.setPower(1);
			}
			else {
				Winch.setPower(0);
				
			}
			
			
			if (gamepad2.dpad_up) {
				ClawLeft.setPosition(CLAW_LEFT_FULL_OPEN_POS);
				ClawRight.setPosition(CLAW_RIGHT_FULL_OPEN_POS);
			}
			
			if (gamepad2.dpad_down) {
				ClawLeft.setPosition(CLAW_LEFT_HALF_CLOSE_POS);
				ClawRight.setPosition(CLAW_RIGHT_HALF_CLOSE_POS);
			}
			
			if (gamepad2.dpad_right) {
				ClawLeft.setPosition(CLAW_LEFT_HALF_CLOSE_POS);
				ClawRight.setPosition(CLAW_RIGHT_FULL_OPEN_POS);
			}
			
			if (gamepad2.dpad_left) {
				ClawLeft.setPosition(CLAW_LEFT_FULL_OPEN_POS);
				ClawRight.setPosition(CLAW_RIGHT_HALF_CLOSE_POS);
			}
			
			if (!prevSpecHotkey) {
				double wristPower = -gamepad2.right_stick_y * WRIST_VELOCITY;
				double powerMult = (gamepad2.right_stick_y > 0 ? 1 : WRIST_LOWER_MULT);
				double holdPower = gamepad2.left_bumper ? -0.025 : 0;
				Wrist.setVelocity( wristPower * powerMult );
			}
			
			if (gamepad2.right_trigger > 0) {
				ClawLeft.setPosition(CLAW_LEFT_CLOSE_POS);
				ClawRight.setPosition(CLAW_RIGHT_CLOSE_POS);
			} else if (gamepad2.left_trigger > 0) {
				ClawLeft.setPosition(CLAW_LEFT_OPEN_POS);
				ClawRight.setPosition(CLAW_RIGHT_OPEN_POS);
			}
			
			// Configured arm+wrist position "hotkeys"
			if (gamepad2.right_bumper && !prevBarHotkey) {
				// Set motor modes to position
				ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				ArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				//Wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				
				// Set motor target positions
				ArmLeft.setPower(1);
				ArmLeft.setVelocity(750);
				ArmLeft.setTargetPosition(BotConfig.BAR_HEIGHT); // Move arm upwards
				ArmRight.setPower(1);
				ArmRight.setVelocity(750);
				ArmRight.setTargetPosition(BotConfig.BAR_HEIGHT);
				//Wrist.setTargetPosition(-650); // Move wrist to face upwards
				//Wrist.setPower(1);
			}
			
			// Configured wrist position "hotkeys"
			if (gamepad2.left_bumper && !prevSpecHotkey) {
				// Set motor modes to position
				Wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				
				// Set motor target positions
				Wrist.setPower(1);
				Wrist.setVelocity(750);
				Wrist.setTargetPosition(BotConfig.SPECIMEN_HEIGHT); // Move wrist to specimen level
			}
			
			// Reset arm and wrist modes if not pressed
			if (!gamepad2.right_bumper && prevBarHotkey) {
				ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			}
			
			// Reset wrist modes if not pressed
			if (!gamepad2.left_bumper && prevSpecHotkey) {
				Wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			}
			
			prevBarHotkey = gamepad2.right_bumper;
			prevSpecHotkey = gamepad2.left_bumper;

			// Telemetry
			// Odometry values
			telemetry.addData("Odometry:", true);
			telemetry.addData("X pos", odometry.getX());
			telemetry.addData("Y pos", odometry.getY());
			telemetry.addData("Heading", odometry.getY());
			telemetry.addLine();

			// Arm values
			telemetry.addData("Arm:", true);
			telemetry.addData("Left arm pos", ArmLeft.getCurrentPosition());
			telemetry.addData("Right arm pos", ArmRight.getCurrentPosition());
			telemetry.addData("Wrist pos", Wrist.getCurrentPosition());
			telemetry.addLine();

			// Sensor values
			telemetry.addData("Sensors:", true);
			telemetry.addData("IMU heading", botHeading);
			telemetry.addData("Distance Sensor", DistSensor.getDistance(DistanceUnit.MM));
			telemetry.addLine();

			telemetry.update();
		}
	}

	/**
	 * if boost trigger unpressed, return base_speed,
	 * else return base_speed + boost amount
	 */
	double calcMaxSpeed(double triggerVal, double BASE_SPEED, double MAX_BOOST) {
		double boostRatio = triggerVal * MAX_BOOST;
		double boostSpeed = boostRatio * BASE_SPEED;
		return BASE_SPEED + boostSpeed;
	}

	double getSavedHeading() {
		Heading heading = new Heading();
		return heading.getHeading();
	}
	
	void SetArmVelocity(double velocity) {
		if ((velocity > 0) || (ArmLeft.getCurrentPosition() > -1600)) {
			ArmLeft.setVelocity(velocity);
			ArmRight.setVelocity(velocity);
		}
		else {
			ArmLeft.setVelocity(0);
			ArmRight.setVelocity(0);
		}
	}
	
}