package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name="IMU+PID Angle Adjustment Test")
public class PIDIMUTest extends LinearOpMode {
    // Constants
    static final double JOYSTICK_ANGLE_MULT = .1;

    // PID constants
    double Kp = 0;
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
		
		// Set zero power behaviours
		BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// Set run mode
		BackLeft.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontLeft.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		FrontRight.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODES);
		BackRight.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Wait for the start button to be pressed
		waitForStart();

		// Reset robot heading on startup (not initialization)
		imu.resetYaw();

		while (opModeIsActive()) {
			// Reset yaw when start button is pressed so that a restart is not needed if the yaw should be reset again.
			if (gamepad1.start) {
				imu.resetYaw();
			}

            double power = pidController.PIDControlRadians(targetAngle, imu.getAngularOrientation().firstAngle);

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

        }
    }
}
