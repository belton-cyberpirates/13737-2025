package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Right", preselectTeleOp="Field Centric (main)")
public class AutoRight extends Auto {
	/**
	 * This function is executed when this Op Mode is initialized from the Driver Station.
	 */
	@Override
	public void runOpMode() {
		Initialize();
		MotorSetup();

		//initAprilTag();
		
		imu.resetYaw();

		waitForStart();

		if (opModeIsActive()) { // <----------------------------------------------------------------
			arm.Move(700, false);
			intake.MoveWrist(650, false);
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 1.2 ));
			arm.Move(0, true);
			intake.OpenClaw(500);
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * .1 ));
			intake.MoveWrist(0, false);
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 1.2 ));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 1 ));
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 0.5 ));
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * 1.6 ));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .2 ));
			driveMotors.Turn((int)(180));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .15 ));
			intake.OpenClaw(500);
			intake.MoveWrist(1400, true);
			intake.CloseClaw(500);
			intake.MoveWrist(0, false);
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 2));
			driveMotors.Turn((int)(180));
			arm.Move(750, false);
			intake.MoveWrist(670, false);
			sleep(500);
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .6 ));
			arm.Move(0, true);
			intake.OpenClaw(500);
			
		}
		saveHeading();
	}
}

