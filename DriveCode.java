package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
	final int BASE_SPEED = 1500;
	final double MAX_BOOST = 0.6; // boost maxes out at an additional 60% of the base speed
	final double STRAFE_MULT = 1.41;

	// Arm constants
	final double ARM_VELOCITY = 1500;
	final double WRIST_POWER = .5;
	final double WRIST_LOWER_MULT = .7;
	final double WRIST_BUTTON_MULT = 1.75;
	
	// Claw constants
	// these are switched for some reason. fix that please.
	final double CLAW_OPEN_POS = 0.9;
	final double CLAW_CLOSE_POS = 0.4;
	

	// Drive motors
	private DcMotorEx BackLeft;
	private DcMotorEx FrontLeft;
	private DcMotorEx FrontRight;
	private DcMotorEx BackRight;

	// Arm motors
	private DcMotorEx ArmLeft;
	private DcMotorEx ArmRight;
	private DcMotorEx Wrist;
	
	// Servos
	private Servo Claw;
	
	private IMU imu;
	
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
		
		// Assign servos
		Claw = hardwareMap.get(Servo.class, "claw");
		
		imu = hardwareMap.get(IMU.class, "imu");
		
		// Set zero power behaviours
		BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		ArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		ArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		Wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		ArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		ArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		Wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// Wait for the start button to be pressed
		waitForStart();

		// Reset robot heading on startup (not initialization)
		imu.resetYaw();
		double savedHeading = getSavedHeading();

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
			double botHeading = (savedHeading + imu .getRobotYawPitchRollAngles() .getYaw(AngleUnit.RADIANS));


			// Virtually rotate the joystick by the negative angle of the robot
			double rotatedX =
				leftStickXGP1 * Math.cos(botHeading) -
				leftStickYGP1 * Math.sin(botHeading);
			double rotatedY =
				leftStickXGP1 * Math.sin(botHeading) +
				leftStickYGP1 * Math.cos(botHeading);
			rotatedX *= STRAFE_MULT; // strafing is slower than rolling, bump speed


			// Set the power of the wheels based off the new joystick coordinates
			// y+x+stick <- [-1,1]
			BackLeft.setVelocity(
				(-rotatedY - rotatedX + rightStickXGP1) * maxSpeed
			);
			FrontLeft.setVelocity(
				(-rotatedY + rotatedX + rightStickXGP1) * maxSpeed
			);
			FrontRight.setVelocity(
				(rotatedY + rotatedX + rightStickXGP1) * maxSpeed
			);
			BackRight.setVelocity(
				(rotatedY - rotatedX + rightStickXGP1) * maxSpeed
			);
			
			SetArmVelocity(gamepad2.left_stick_y * ARM_VELOCITY);
			
			double wristPower = -gamepad2.right_stick_y * WRIST_POWER;
			double powerMult = (gamepad2.right_stick_y > 0 ? 1 : WRIST_LOWER_MULT);
			double buttonMult = gamepad2.right_bumper ? WRIST_BUTTON_MULT : 1;
			double holdPower = gamepad2.left_bumper ? -0.025 : 0;
			Wrist.setPower(( wristPower * powerMult * buttonMult ) + holdPower);
			
			if (gamepad2.right_trigger > 0) {
				Claw.setPosition(CLAW_OPEN_POS);
			} else if (gamepad2.left_trigger > 0) {
				Claw.setPosition(CLAW_CLOSE_POS);
			}

			// Telemetry
			telemetry.addData("Left arm pos", ArmLeft.getCurrentPosition());
			telemetry.addData("Right arm pos", ArmRight.getCurrentPosition());

			telemetry.update();
		}
	}

	/**
	 * if boost trigger unpressed, return base_speed,
	 * else return base_speed + boost amount
	 */
	double calcMaxSpeed(double triggerVal, int BASE_SPEED, double MAX_BOOST) {
		double boostRatio = triggerVal * MAX_BOOST;
		double boostSpeed = boostRatio * BASE_SPEED;
		return BASE_SPEED + boostSpeed;
	}

	double getSavedHeading() {
		Heading heading = new Heading();
		return heading.getHeading();
		/*
		try {
			BufferedReader br = new BufferedReader(new FileReader("heading.txt"));
			String line = br.readLine();

			br.close();

			telemetry.addData("Retrieved saved heading:", true);
			
			return Double.parseDouble(line);
		} catch(Exception e) {
			telemetry.addData("Retrieved saved heading:", false);
			return 0d;
		}*/
	}
	
	void SetArmVelocity(double velocity) {
		ArmLeft.setVelocity(velocity);
		ArmRight.setVelocity(-velocity);
	}
	
}
