package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Specimens - Park")
public class AutoRightPark extends Auto {

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
			
			// Grab next block
			new Move(this, BotConfig.BLOCK_GRAB_X, BotConfig.FIRST_BLOCK_Y, 0),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT, true),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 250),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT - 50, false),
			
			// Place block in zone
			new Move(this, 400, BotConfig.FIRST_BLOCK_Y, -175),
			new OpenClawWide(this),
			new MoveWrist(this, 0, false),
			
			// Push second block
			new Move(this, BotConfig.PLOW_X, BotConfig.FIRST_BLOCK_Y, 180),
			new Move(this, BotConfig.PLOW_X, BotConfig.SECOND_BLOCK_Y, 90),
			new Move(this, 300, BotConfig.SECOND_BLOCK_Y, 90),
			
			// Pick up 2nd specimen
			new Move(this, BotConfig.PICKUP_X, BotConfig.PICKUP_Y, 135),
			new MoveWrist(this, BotConfig.WRIST_SPECIMEN_HEIGHT, true),
			new Wait(this, BotConfig.HUMAN_WAIT_TIME),
			new CloseClaw(this),
			new Wait(this, 300),
			
			// Score 2nd specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Move(this, BotConfig.BAR_X, 100, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+400, false),
			new Move(this, BotConfig.BAR_SCORE_X, 100, 0),
			new OpenClawWide(this),
			
			// Pick up 3rd specimen
			new Move(this, BotConfig.PICKUP_X, BotConfig.PICKUP_Y, 135),
			new MoveWrist(this, BotConfig.WRIST_SPECIMEN_HEIGHT, true),
			new Wait(this, BotConfig.HUMAN_WAIT_TIME),
			new CloseClaw(this),
			new Wait(this, 300),
			
			// Score 3rd specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Move(this, BotConfig.BAR_X, 200, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+400, false),
			new Move(this, BotConfig.BAR_SCORE_X, 200, 0),
			new OpenClawWide(this),
			
			new Wait(this, 250),
			
			new Move(this, 100, -1200, 0),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

