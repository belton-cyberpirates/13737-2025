package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Place One", preselectTeleOp="Field Centric (main)")
public class AutoOneSpecimen extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			// Score first specimen
			new MoveWrist(this, BotConfig.WRIST_BAR_READY_HEIGHT, false),
			new Move(this, 0, 550, 0),
			new MoveArm(this, BotConfig.BAR_HEIGHT, true),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Wait(this, 500),
			new MoveArm(this, 0, false),
			new Wait(this, 750),
			new OpenClaw(this),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

