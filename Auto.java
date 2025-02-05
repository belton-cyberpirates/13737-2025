package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Heading;


import java.util.List;

public abstract class Auto extends LinearOpMode {
	public protected DriveMotors driveMotors;
	public protected Arm arm;
	public protected Intake intake;
	public protected IMU imu;
	public protected Heading heading;
	
	/**
	 * Initialize classes used by autos
	 */
	protected void Initialize() {
		driveMotors = new DriveMotors(this);
		arm = new Arm(this);
		intake = new Intake(this);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.resetYaw();
		telemetry.addData("Beginning Initialization...", false);
		telemetry.update();
	}

	/**
	 * Set reliable initial configuration for robot motors
	 */
	protected void MotorSetup() {
		intake.CloseClaw(0);
		intake.DropWrist();
		arm.DropArm();
		sleep(5000);
		arm.Initialize();
		intake.InitializeWrist();
		telemetry.addData("Fully Initialized", true);
		telemetry.update();
	}
	
	protected void rotateTo(double deg) {
		double _heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
		driveMotors.Turn((int)(_heading-deg));
	}

	protected void saveHeading() {
		double _heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
		this.heading.setHeading(_heading);
	}
}
