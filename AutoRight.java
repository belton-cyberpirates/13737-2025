package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Specimens")
public class AutoRight extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //
			
			// Score first specimen
			new Move(this, BotConfig.BAR_X, 0, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, true),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+400, false),
			new Move(this, BotConfig.BAR_X-450, 0, 0),
			new OpenClaw(this),
			
			// Grab next block
			new Move(this, BotConfig.BLOCK_GRAB_X, BotConfig.FIRST_BLOCK_Y, 0),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT, true),
			new CloseClaw(this),
			new Wait(this, 250),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT - 200, false),
			
			// Place block in zone
			new Move(this, 400, BotConfig.FIRST_BLOCK_Y, -175),
			new OpenClaw(this),
			
			new Wait(this, 1000),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

