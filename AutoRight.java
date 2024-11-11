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

		waitForStart();

		if (opModeIsActive()) { // <----------------------------------------------------------------
			driveMotors.Move(Direction.RIGHT, (int)(BotConfig.TILE_LENGTH * 2));
		}
		saveHeading();
	}
}

