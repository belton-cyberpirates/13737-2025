package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Specimens - Four", group="1")
public class AutoRightFour extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //
			
			// Score first specimen
			new MoveDifferential(this, 0, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Move(this, BotConfig.BAR_X, 0, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+300, false),
			new Wait(this, 100),
			new Move(this, BotConfig.BAR_SCORE_X, 0, 0),
			new OpenClawWide(this),
			
			// Reset wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new Wait(this, 250),
			
			// Grab first block
			new Move(this, BotConfig.BLOCK_GRAB_X, BotConfig.FIRST_BLOCK_Y, 0),
			new MoveWristWithVelocity(this, BotConfig.WRIST_SAMPLE_HEIGHT, true, BotConfig.WRIST_VELOCITY / 1.5),
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
			new MoveDifferential(this, 100, 0),
			new Move(this, BotConfig.PICKUP_X, BotConfig.PICKUP_Y, 180),
			new MoveWrist(this, BotConfig.WRIST_SPECIMEN_HEIGHT, true),
			new Wait(this, BotConfig.HUMAN_WAIT_TIME),
			new CloseClaw(this),
			new Wait(this, 300),
			
			// Score 2nd specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new MoveDifferential(this, 0, 0),
			new Move(this, BotConfig.BAR_X, 100, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+300, false),
			new Wait(this, 100),
			new Move(this, BotConfig.BAR_SCORE_X, 100, 0),
			new OpenClawWide(this),
			
			// Reset wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new Wait(this, 250),
			
			// Pick up 3rd specimen
			new MoveDifferential(this, 100, 0),
			new Move(this, BotConfig.PICKUP_X, BotConfig.PICKUP_Y, 180),
			new MoveWrist(this, BotConfig.WRIST_SPECIMEN_HEIGHT, true),
			new Wait(this, BotConfig.HUMAN_WAIT_TIME),
			new CloseClaw(this),
			new Wait(this, 300),
			
			// Score 3rd specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new MoveDifferential(this, 0, 0),
			new Move(this, BotConfig.BAR_X, 200, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+300, false),
			new Wait(this, 100),
			new Move(this, BotConfig.BAR_SCORE_X, 200, 0),
			new OpenClawWide(this),
			
			// Reset wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new Wait(this, 250),
			
			// Pick up 4th specimen
			new MoveDifferential(this, 100, 0),
			new Move(this, BotConfig.PICKUP_X, BotConfig.PICKUP_Y, 180),
			new MoveWrist(this, BotConfig.WRIST_SPECIMEN_HEIGHT, true),
			new Wait(this, BotConfig.HUMAN_WAIT_TIME),
			new CloseClaw(this),
			new Wait(this, 300),
			
			// Score 4th specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new MoveDifferential(this, 0, 0),
			new Move(this, BotConfig.BAR_X, 300, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+300, false),
			new Wait(this, 100),
			new Move(this, BotConfig.BAR_SCORE_X, 300, 0),
			new OpenClawWide(this),
			
			// Reset wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new Wait(this, 250),
			
			new Wait(this, 250),
			new Move(this, 100, -1300, 0),
			
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

