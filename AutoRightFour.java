package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Specimens - 4", group="1")
public class AutoRightFour extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //
			
			// Score first
			new MoveDifferential(this, 110, 0),
			new MoveWristWithVelocity(this, 1350, false, BotConfig.WRIST_VELOCITY),
			new Wait(this, 250),
			new Move(this, 300, -250, -35, .75),
			new Move(this, 550, 0, -35),
			
			new MoveWristWithVelocity(this, 1125, true, BotConfig.WRIST_VELOCITY),
			new Wait(this, 250),
			new Move(this, 300, -100, -35, 1),
			new OpenClawWide(this),
			
			
			// Plow first block
			new MoveWrist(this, 20, false),
			new Wait(this, 700),
			new Move(this, 300, -600, 0, .75),
			new Move(this, BotConfig.PLOW_X, -600, 0, 1),
			new Move(this, BotConfig.PLOW_X, BotConfig.FIRST_BLOCK_Y, 0, .75),
			new Move(this, 200, BotConfig.FIRST_BLOCK_Y, 0, 1),
			
			// Plow second block
			new Move(this, BotConfig.PLOW_X, -600, 0, 1),
			new Move(this, BotConfig.PLOW_X, BotConfig.SECOND_BLOCK_Y, 0, .75),
			new Move(this, 200, BotConfig.SECOND_BLOCK_Y, 0, 1),
			
			
			// Grab second
			new MoveDifferential(this, 150, 0),
			new Move(this, 100, BotConfig.PICKUP_Y, 0),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 400),
			
			// Score second
			new MoveDifferential(this, 110, 0),
			new MoveWristWithVelocity(this, 1350, false, BotConfig.WRIST_VELOCITY),
			new Wait(this, 500),
			new Move(this, 300, -250, -35, .75),
			new Move(this, 550, 0, -35),
			
			new MoveWristWithVelocity(this, 1125, true, BotConfig.WRIST_VELOCITY),
			new Wait(this, 250),
			new Move(this, 300, -100, -35, 1),
			new OpenClawWide(this),
			new MoveWrist(this, 20, false),
			
			
			// Grab third
			new MoveDifferential(this, 150, 0),
			new Move(this, 100, BotConfig.PICKUP_Y, 0),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 400),
			
			// Score third
			new MoveDifferential(this, 110, 0),
			new MoveWristWithVelocity(this, 1350, false, BotConfig.WRIST_VELOCITY),
			new Wait(this, 500),
			new Move(this, 300, -225, -35, .75),
			new Move(this, 550, 0, -35),
			
			new MoveWristWithVelocity(this, 1125, true, BotConfig.WRIST_VELOCITY),
			new Wait(this, 250),
			new Move(this, 300, -100, -35, 1),
			new OpenClawWide(this),
			new MoveWrist(this, 20, false),
			
			
			// Grab fourth
			new MoveDifferential(this, 150, 0),
			new Move(this, 100, BotConfig.PICKUP_Y, 0),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 400),
			
			// Score fourth
			new MoveDifferential(this, 110, 0),
			new MoveWristWithVelocity(this, 1350, false, BotConfig.WRIST_VELOCITY),
			new Wait(this, 500),
			new Move(this, 300, -200, -35, .75),
			new Move(this, 550, 0, -35),
			
			new MoveWristWithVelocity(this, 1125, true, BotConfig.WRIST_VELOCITY),
			new Wait(this, 250),
			new Move(this, 300, -100, -35, 1),
			new OpenClawWide(this),
			new MoveWrist(this, 20, false),
			
			// Park
			new Move(this, 0, -1000, 0),
			
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

