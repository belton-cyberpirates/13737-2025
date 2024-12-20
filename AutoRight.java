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
			// score first specimen
			arm.Move(BotConfig.BAR_HEIGHT, false);
			intake.MoveWrist(750, false);
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 1.2 ));
			arm.Move(0, true);
			intake.OpenClaw(500);
			// back up and turn
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * .1 ));
			intake.MoveWrist(0, false);
			driveMotors.Turn(90);
			// move to get in front of first block
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 1.2 ));
			driveMotors.Move(Direction.LEFT, (int)( BotConfig.TILE_LENGTH * 1.2 ));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * 0.5 ));
			// get sample in observation zone
			driveMotors.Turn(-90);
			driveMotors.Move(Direction.BACKWARD, (int)( BotConfig.TILE_LENGTH * 1.5 ));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .2 ));
			driveMotors.Turn((int)(180));
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .25 ));
			intake.OpenClaw(500);
			intake.MoveWrist(1400, true);
			intake.CloseClaw(500);
			intake.MoveWrist(0, false);
			driveMotors.Move(Direction.RIGHT, (int)( BotConfig.TILE_LENGTH * 2));
			driveMotors.Turn((int)(180));
			arm.Move(BotConfig.BAR_HEIGHT, false);
			intake.MoveWrist(700, false);
			sleep(500);
			driveMotors.Move(Direction.FORWARD, (int)( BotConfig.TILE_LENGTH * .6 ));
			arm.Move(0, true);
			intake.OpenClaw(500);
			
		}
		saveHeading();
	}
}

