package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "AutoRedLeft", preselectTeleOp="Field Centric (main)")
public class AutoRedLeft extends Auto {
	/**
	 * This function is executed when this Op Mode is initialized from the Driver Station.
	 */
	@Override
	public void runOpMode() {
		Initialize();

		waitForStart();

		if (opModeIsActive()) { // <----------------------------------------------------------------
			// strafe to avoid submersible
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * 1));
			// score block 1
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 2.1));
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * .5));
	  		driveMotors.Move(Direction.BACKWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
			// score block 2
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * .5));
	  		driveMotors.Move(Direction.BACKWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
			// score block 3
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * .5));
	  		driveMotors.Move(Direction.BACKWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		// park/low ascent 
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 2));
	  		driveMotors.Turn(90);
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 1.7));
		}
		saveHeading();
	}
}

