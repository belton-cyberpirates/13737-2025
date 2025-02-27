package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Score First Specimen", preselectTeleOp="Field Centric (main)")
public class AutoOneSpecimen extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //
			
			new Move(this, BotConfig.BAR_X, 0, 0),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, true),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT+400, false),
			new Move(this, BotConfig.BAR_X-450, 0, 0),
			new OpenClaw(this),
			new MoveWrist(this, 0, false),
			
			new Wait(this, 1000),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

