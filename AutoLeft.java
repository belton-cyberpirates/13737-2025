package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Baskets", group="1")
public class AutoLeft extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //
			
			// Score starting sample
			new Move(this, 85, 830, -115, 2),
			new MoveArm(this, BotConfig.BASKET_HEIGHT, true, 0.25),
			new MoveWrist(this, BotConfig.WRIST_DUNK_HEIGHT, true),
			new OpenClawWide(this),
			new Wait(this, 250),
			
			// Reset arm and wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new MoveArmWithVelocity(this, 0, true, BotConfig.ARM_VELOCITY/4),
			
			// Grab 2nd sample
			new Move(this, BotConfig.BLOCK_GRAB_X, -BotConfig.FIRST_BLOCK_Y, 0, 2),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT, true),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 250),
			new MoveWrist(this, 0, false),
			
			// Score 2nd sample
			new Move(this, BotConfig.BASKET_X, BotConfig.BASKET_Y, -135, 2),
			new MoveArm(this, BotConfig.BASKET_HEIGHT, true, 0.25),
			new MoveWrist(this, BotConfig.WRIST_DUNK_HEIGHT, true),
			new OpenClawWide(this),
			new Wait(this, 250),
			
			// Reset arm and wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new MoveArmWithVelocity(this, 0, true, BotConfig.ARM_VELOCITY/4),
			
			// Grab 3rd sample
			new Move(this, BotConfig.BLOCK_GRAB_X, -BotConfig.SECOND_BLOCK_Y, 0, 2),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT, true),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 250),
			new MoveWrist(this, 0, false),
			new Wait(this, 500),
			
			// Score 3rd sample 
			new Move(this, BotConfig.BASKET_X, BotConfig.BASKET_Y, -135, 2),
			new MoveArm(this, BotConfig.BASKET_HEIGHT, true, 0.25),
			new MoveWrist(this, BotConfig.WRIST_DUNK_HEIGHT, true),
			new OpenClawWide(this),
			new Wait(this, 250),
			
			// Reset arm and wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, false),
			new MoveArmWithVelocity(this, 0, true, BotConfig.ARM_VELOCITY/4),
			
			// Grab 4th sample
			new Move(this, BotConfig.THIRD_BLOCK_X, -BotConfig.THIRD_BLOCK_Y - 100, -90, 4),
			new MoveWrist(this, BotConfig.WRIST_SIDE_SAMPLE_HEIGHT, true),
			new Move(this, BotConfig.THIRD_BLOCK_X, -BotConfig.THIRD_BLOCK_Y, -90, 2),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 250),
			new Move(this, BotConfig.THIRD_BLOCK_X, -BotConfig.THIRD_BLOCK_Y - 150, -90, 1),
			new MoveWrist(this, 0, false),
			new Wait(this, 500),
			
			// Score 4th sample 
			new Move(this, BotConfig.BASKET_X, BotConfig.BASKET_Y, -135, 2),
			new MoveArm(this, BotConfig.BASKET_HEIGHT, true, 0.25),
			new MoveWrist(this, BotConfig.WRIST_DUNK_HEIGHT, true),
			new OpenClawWide(this),
			new Wait(this, 250),
			
			// Reset arm and wrist
			new MoveWrist(this, BotConfig.WRIST_PASSIVE, true),
			new MoveArmWithVelocity(this, 0, true, BotConfig.ARM_VELOCITY/4),
			
			// Preform a level 1 ascent
			new MoveWrist(this, 800, false),
			new Move(this, 1650, 580, 90)
			
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

