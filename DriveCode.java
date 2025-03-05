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
	final double MAX_BOOST = 0.66; // boost maxes out at an additional 60% of the base speed
	final double STRAFE_MULT = 1.2;
	final double TURN_MULT = 1.2;

	// Arm constants
	final double ARM_VELOCITY = 750;
	final double WRIST_VELOCITY = 1500;
	final double WRIST_LOWER_MULT = .7;
	final double WRIST_BUTTON_MULT = 1.75;
	
	// Winch Constants
	final double WINCH_POWER = 1;
	
	private DcMotorEx Winch;

	DriveMotors driveMotors;
	Arm arm;
	Intake intake;

	@Override
	public void runOpMode() throws InterruptedException {
		
		driveMotors = new DriveMotors(this);
		arm = new Arm(this);
		intake = new Intake(this);

		// Create winch motor
		Winch = hardwareMap.get(DcMotorEx.class, "winch");
		
		// Set winch motor behavior
		Winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Wait for the start button to be pressed
		waitForStart();

		double savedHeading = getSavedHeading();
		
		boolean hanging = false;

		while (opModeIsActive()) {
			// Reset yaw when back button pressed so restarting is not needed if it needs a reset
			if (gamepad1.y) {
				driveMotors.odometry.recalibrateIMU();
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
			double botHeading = driveMotors.odometry.getHeading();


			// Virtually rotate the joystick by the angle of the robot
			double rotatedX =
				leftStickXGP1 * Math.cos(botHeading) -
				leftStickYGP1 * Math.sin(botHeading);
			double rotatedY =
				leftStickXGP1 * Math.sin(botHeading) +
				leftStickYGP1 * Math.cos(botHeading);
			
			// strafing is slower than rolling, bump horizontal speed
			rotatedX *= STRAFE_MULT;
			
			
			if (gamepad1.a) {
				driveMotors.Move(BotConfig.PICKUP_X, BotConfig.PICKUP_Y, 135);
			}
			else {
				// Set the power of the wheels based off the new joystick coordinates
				// y+x+stick <- [-1,1]
				driveMotors.DriveWithPower(
					( rotatedY + rotatedX - ( rightStickXGP1 * TURN_MULT )) * maxSpeed, // Back left
					( rotatedY - rotatedX - ( rightStickXGP1 * TURN_MULT )) * maxSpeed, // Front left
					(-rotatedY - rotatedX - ( rightStickXGP1 * TURN_MULT )) * maxSpeed, // Front right
					(-rotatedY + rotatedX - ( rightStickXGP1 * TURN_MULT )) * maxSpeed  // Back right
				);
			}
			
			
			// Winch code
			if (gamepad1.dpad_up) {
				hanging = true;
			}
			
			if (gamepad1.dpad_down) {
				Winch.setPower(-WINCH_POWER);
				hanging = false;
			}
			else if (hanging) {
				Winch.setPower(WINCH_POWER);
			}
			else {
				Winch.setPower(0);
			}
			
			
			// Claw code
			// Open
			if (gamepad2.right_trigger > 0) {
				intake.CloseClaw();
			} 
			// Close
			if (gamepad2.left_trigger > 0) {
				intake.OpenClaw();
			}
			// Wide open
			if (gamepad2.dpad_up) {
				intake.SetClawPos(BotConfig.CLAW_LEFT_FULL_OPEN_POS, BotConfig.CLAW_RIGHT_FULL_OPEN_POS);
			}
			// Half open
			if (gamepad2.dpad_down) {
				intake.SetClawPos(BotConfig.CLAW_LEFT_HALF_CLOSE_POS, BotConfig.CLAW_RIGHT_HALF_CLOSE_POS);
			}
			// Left only
			if (gamepad2.dpad_right) {
				intake.SetClawPos(BotConfig.CLAW_LEFT_HALF_CLOSE_POS, BotConfig.CLAW_RIGHT_FULL_OPEN_POS);
			}
			// Right only
			if (gamepad2.dpad_left) {
				intake.SetClawPos(BotConfig.CLAW_LEFT_FULL_OPEN_POS, BotConfig.CLAW_RIGHT_HALF_CLOSE_POS);
			}
			

			// Arm code
			// High chamber hotkey
			if (gamepad2.right_stick_button) {
				arm.Move(BotConfig.BASKET_SAFE_HEIGHT);
				arm.setVelocity((int)( ARM_VELOCITY * 2 ));
			}
			else {
				SetArmVelocity(gamepad2.left_stick_y * ARM_VELOCITY);
			}
			

			// Wrist code
			// Specimen pickup hotkey
			if (gamepad2.right_stick_button) {
				intake.MoveWrist(BotConfig.WRIST_BASKET_SAFE_HEIGHT);
				intake.wrist.setVelocity(BotConfig.WRIST_VELOCITY);
			}
			else if (gamepad2.right_bumper) {
				intake.MoveWrist(BotConfig.WRIST_SPECIMEN_HEIGHT);
				intake.wrist.setVelocity(BotConfig.WRIST_VELOCITY * 2);
			}
			else {
				double wristPower = -gamepad2.right_stick_y * WRIST_VELOCITY;
				double powerMult = gamepad2.right_stick_y > 0 ? 1 : WRIST_LOWER_MULT;
				double holdPower = gamepad2.left_bumper ? -1 : 0;
				intake.MoveWristWithVelocity( (wristPower * powerMult) + holdPower );
			}
			

			// Process classes
			driveMotors.process();
			arm.process();


			// Telemetry
			// Odometry values
			telemetry.addData("X pos", driveMotors.odometry.getPosX());
			telemetry.addData("Y pos", driveMotors.odometry.getPosY());
			telemetry.addData("Heading", driveMotors.odometry.getHeading());

			// Arm + intake values
			telemetry.addData("Arm pos", arm.getheight());
			telemetry.addData("Wrist pos", intake.wrist.getCurrentPosition());

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
		if ((velocity > 0) || (this.arm.getHeight() < BotConfig.MAX_ARM_HEIGHT)) {
			arm.MoveWithVelocity(velocity);
		}
		else {
			arm.MoveWithPower(-.1);
		}
	}
}