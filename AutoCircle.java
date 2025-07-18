package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Circle", group="3")
public class AutoCircle extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			new Move(this, 0, 0, 90),
			new Move(this, 0, 1200, 0),
			new Move(this, 2400, 1200, 0),
			new Move(this, 2400, -1200, 0),
			new Move(this, 0, -1200, 0),
			new Move(this, 0, 0, 0),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

