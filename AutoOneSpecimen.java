package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Score Specimen", preselectTeleOp="Field Centric (main)", group="2")
public class AutoOneSpecimen extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //
			
			// Score first specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Move(this, BotConfig.BAR_X, 0, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+400, false),
			new Wait(this, 100),
			new Move(this, BotConfig.BAR_SCORE_X, 0, 0),
			new OpenClawWide(this),
			
			// Reset wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new Wait(this, 250),
			
			// Move out of the way
			new Move(this, BotConfig.BLOCK_GRAB_X, -BotConfig.FIRST_BLOCK_Y, 0),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

