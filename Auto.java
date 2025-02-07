package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Arrays;
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


public abstract class Auto extends LinearOpMode {
	abstract Action[] getActions();

	public DriveMotors driveMotors;
	public Arm arm;
	public Intake intake;
	public IMU imu;
	public Heading heading;
	
	/**
	 * Initialize classes used by autos
	 */
	protected void Initialize() {
		imu = hardwareMap.get(IMU.class, BotConfig.IMU_NAME);

		driveMotors = new DriveMotors(this);
		arm = new Arm(this);
		intake = new Intake(this);

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
		sleep(3000);
		arm.Initialize();
		intake.InitializeWrist();
		telemetry.addData("Fully Initialized", true);
		telemetry.update();
	}

	protected void saveHeading() {
		double _heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
		this.heading.setHeading(_heading);
	}

	@Override
	public void runOpMode() {
		Initialize();
		MotorSetup();

		waitForStart();

		Action[] actions = getActions();

		while (opModeIsActive() && ( actions.length > 0 )) { // <----------------------------------------------------------------
			if ( actions[0].isDone() ) {
				actions = Arrays.copyOfRange(actions, 1, actions.length);
				actions[0].onStart();
			}
		}

		saveHeading();
	}


	public double getHeading() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
	}
}
