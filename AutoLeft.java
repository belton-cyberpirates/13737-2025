package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Baskets")
public class AutoLeft extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //
			
			// Score starting sample
			new Move(this, 85, 830, -115),
			new MoveArm(this, BotConfig.BASKET_HEIGHT, true),
			new MoveWrist(this, 800, true),
			new OpenClawWide(this),
			new Wait(this, 250),
			
			// Reset arm and wrist
			new MoveWrist(this, 0, false),
			new MoveArm(this, 0, true),
			
			// Grab 2nd sample
			new Move(this, BotConfig.BLOCK_GRAB_X, -BotConfig.FIRST_BLOCK_Y, 0),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT, true),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 250),
			new MoveWrist(this, 0, false),
			
			// Score 2nd sample block
			new Move(this, BotConfig.BASKET_X, BotConfig.BASKET_Y, -135),
			new MoveArm(this, BotConfig.BASKET_HEIGHT, true),
			new MoveWrist(this, 800, true),
			new OpenClawWide(this),
			new Wait(this, 250),
			
			// Reset arm and wrist
			new MoveWrist(this, 0, false),
			new MoveArm(this, 0, true),
			
			// Grab 3rd sample
			new Move(this, BotConfig.BLOCK_GRAB_X, -BotConfig.SECOND_BLOCK_Y, 0),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT, true),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 250),
			new MoveWrist(this, 0, false),
			new Wait(this, 500),
			
			// Score 3rd sample block
			new Move(this, BotConfig.BASKET_X, BotConfig.BASKET_Y, -135),
			new MoveArm(this, BotConfig.BASKET_HEIGHT, true),
			new MoveWrist(this, 800, true),
			new OpenClawWide(this),
			new Wait(this, 250),
			
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

