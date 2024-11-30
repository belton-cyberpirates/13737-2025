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
			arm.Move(730, false);
			intake.MoveWrist(570, false);
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 1.22 ));
			arm.Move(300, true);
			intake.OpenClaw(500);
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * .1 ));
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 1.2 ));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 1 ));
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 0.5 ));
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * 1.7 ));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .2 ));
			driveMotors.Turn((int)(180));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .15 ));
			intake.MoveWrist(1250, true);
			intake.CloseClaw(500);
			intake.MoveWrist(0, true);
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 2));
			driveMotors.Turn((int)(180));
			arm.Move(780, false);
			intake.MoveWrist(570, false);
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .75 ));
			arm.Move(300, true);
			intake.OpenClaw(500);
			
		}
		saveHeading();
	}
}
