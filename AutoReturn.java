package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Return", preselectTeleOp="Odometry Test Right", group="3")
public class AutoReturn extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			// ======================= AUTO START ======================= //

			new Wait(this, 20000),
			new Move(this, 0, 0, 0),
			new Wait(this, 5000),
			
			// ======================== AUTO END ======================== //
		};
		
		return actions;
	}
}

