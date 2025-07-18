package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Forward", group="3")
public class AutoForward extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			new Move(this, 1000, 0, 0),
			new Move(this, 0, 0, 0),
			new Wait(this, 1000),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

