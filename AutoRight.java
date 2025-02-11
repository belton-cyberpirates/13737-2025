package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Sigma Auto Right"/*, preselectTeleOp="Field Centric (main)"*/)
public class AutoRight extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			// Score first specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Move(this, 0, 550, 0),
			new MoveArm(this, BotConfig.BAR_HEIGHT, true),
			new MoveWrist(this, 1000, false),
			new Wait(this, 500),
			new MoveArm(this, 0, false),
			new Wait(this, 750),
			
			// Reset wrist
			new OpenClaw(this),
			new MoveWrist(this, 0, false),
			new Wait(this, 500),
			
			// Grab first block
			new Move(this, 1000, 550, 0),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT, true),
			new CloseClaw(this),
			new Wait(this, 500),
			new MoveWrist(this, BotConfig.WRIST_SAMPLE_HEIGHT - 100, true),
			
			// Drop first block into observation zone
			new Move(this, 1000, 550, 180),
			new Wait(this, 5000),
			new OpenClaw(this),
			new Wait(this, 500),
			new MoveWrist(this, 0, true),
			new Wait(this, 3000),
			
			// Grab newly made specimen
			new MoveWrist(this, BotConfig.WRIST_SPECIMEN_HEIGHT, true),
			new Wait(this, 250),
			new CloseClaw(this),
			new Wait(this, 250),
			new MoveWrist(this, 0, false),
			
			// Move to sumbersible
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Move(this, 100, 550, 180),
			new Move(this, 100, 550, 0),
			new MoveArm(this, BotConfig.BAR_HEIGHT, true),
			new MoveWrist(this, 1000, false),
			new Wait(this, 500),
			new MoveArm(this, 0, false),
			new Wait(this, 750),
			
			
			new Wait(this, 50000),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

