package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.time.Duration;
import java.time.Instant;

import org.firstinspires.ftc.teamcode.PIDController;

enum PIDEnum {
	KP = 0,
	KI = 1,
	KD = 2
}

@TeleOp(name="PIDAngleTest")
public class PIDIMUTest extends LinearOpMode {
	// Constants
	static final double JOYSTICK_ANGLE_MULT = .1;

	// PID constants
	double Kp = .1;
	double Ki = 0;
	double Kd = 0;

	// Drive motors
	private DcMotorEx BackLeft;
	private DcMotorEx FrontLeft;
	private DcMotorEx FrontRight;
	private DcMotorEx BackRight;
	
	// Sensors
	private IMU imu;

	// PID controller
	PIDController pidController = new PIDController(Kp, Ki, Kd);

	// Vars
	double targetAngle = 0;

	@Override
	public void runOpMode() throws InterruptedException {
		// Assign drive motors
		BackLeft = hardwareMap.get(DcMotorEx.class, "back_left");
		FrontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
		FrontRight = hardwareMap.get(DcMotorEx.class, "front_right");
		BackRight = hardwareMap.get(DcMotorEx.class, "back_right");
		
		// Assign sensors
		imu = hardwareMap.get(IMU.class, "imu");

		// Init IMU
		// IMU.Parameters parameters = new IMU.Parameters();
		// parameters.mode = IMU.SensorMode.IMU;
		// parameters.angleUnit = IMU.AngleUnit.RADIANS;
		// imu.initialize();
		
		// Set zero power behaviours
		BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// Set run mode
		BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Wait for the start button to be pressed
		waitForStart();

        ElapsedTime timer = new ElapsedTime();
		PIDEnum targetConstant = PIDEnum.KP;

		boolean prevPressed = false;

		// Reset robot heading on startup (not initialization)
		imu.resetYaw();
		
		while (opModeIsActive()) {
			
			// Reset yaw when start button is pressed so that a restart is not needed if the yaw should be reset again.
			if (gamepad1.start) {
				imu.resetYaw();
			}

			double currentAngle = imu.getRobotYawPitchRollAngles().getYaw();
			double power = pidController.PIDControlRadians(targetAngle, currentAngle, timer.seconds());
            timer.reset();

			BackLeft.setPower(power);
			FrontLeft.setPower(power);
			FrontRight.setPower(power);
			BackRight.setPower(power);

			targetAngle += gamepad1.right_stick_x * JOYSTICK_ANGLE_MULT;

			if (gamepad1.dpad_up) {
				targetAngle = 0;
			}
			else if (gamepad1.dpad_right) {
				targetAngle = Math.PI * 0.5;
			}
			else if (gamepad1.dpad_down) {
				targetAngle = Math.PI;
			}
			else if (gamepad1.dpad_left) {
				targetAngle = Math.PI * 1.5;
			}

			if (gamepad2.dpad_up) {
				double val = 1 - gamepad2.right_trigger;

				switch (targetConstant) {
					case PIDEnum.KP:
						Kp += val;
						break;
					
					case PIDEnum.KI:
						Ki += val;
						break;
					
					case PIDEnum.KD:
						Kd += val;
						break;
				}
			}
			else if (gamepad2.dpad_down) {
				double val = 1 - gamepad2.right_trigger;
				
				switch (targetConstant) {
					case PIDEnum.KP:
						Kp += val;
						break;
					
					case PIDEnum.KI:
						Ki += val;
						break;
					
					case PIDEnum.KD:
						Kd += val;
						break;
				}
			}
			
			if (gamepad2.dpad_left && !prevPressed) {
				int newTarget = (targetConstant - 1) % 3;
				targetConstant = newTarget >= 0 ? newTarget : newTarget + 3;
			}
			else if (gamepad2.dpad_right && !prevPressed) {
				targetConstant = (targetConstant + 1) % 3;
			}

			prevPressed = gamepad2.dpad_left || gamepad2.dpad_right;

			telemetry.addData("Current angle", currentAngle);
			telemetry.addData("Target angle", targetAngle);
			telemetry.addData("Power", power);

			String targ = "";
			switch (targetConstant) {
				case PIDEnum.KP:
					targ = "Kp";
					break;
				
				case PIDEnum.KI:
					targ = "Ki";
					break;
				
				case PIDEnum.KD:
					targ = "Kd";
					break;
			}

			telemetry.addData("Target Value", targ);
			telemetry.addData("Kp", Kp);
			telemetry.addData("Ki", Ki);
			telemetry.addData("Kd", Kd);

			telemetry.update();
        }
	}
}
