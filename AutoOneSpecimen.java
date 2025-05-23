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
			
			// Score first
			new MoveDifferential(this, 110, 0),
			new MoveWristWithVelocity(this, 1300, false, BotConfig.WRIST_VELOCITY),
			new Move(this, 300, -250, -35, .75),
			new Move(this, 550, 0, -35),
			
			new MoveWristWithVelocity(this, 1125, true, BotConfig.WRIST_VELOCITY),
			new Wait(this, 250),
			new Move(this, 300, -100, -35, 1),
			new OpenClawWide(this),
			
			// Reset wrist
			new MoveWrist(this, 20, false),
			new Wait(this, 700),
			
			// Park
			new Move(this, 0, -1000, 0),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

