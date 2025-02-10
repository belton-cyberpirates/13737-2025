package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Right"/*, preselectTeleOp="Field Centric (main)"*/)
public class AutoRight extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			// Score first specimen
			new MoveArm(this, BotConfig.BAR_HEIGHT, false),
			new MoveWrist(this, BotConfig.WRIST_BAR_HEIGHT, false),
			new Move(this, 0, 550, 0),
			new Wait(this, 1000),
			new MoveWrist(this, 1000, false),
			new Wait(this, 500),
			new MoveArm(this, 0, false),
			new Wait(this, 750),
			new OpenClaw(this),
			
			// Move to first block
			new MoveWrist(this, 0, false),
			new Wait(this, 500),
			new Move(this, 650, 500, 180),
			new Move(this, 650, 1250, 180),
			
			//new Wait(this, 50000),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

