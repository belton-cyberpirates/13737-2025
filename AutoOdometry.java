package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Odometry", preselectTeleOp="Field Centric (main)")
public class AutoOdometry extends Auto {

	public Action[] getActions() {
		Action[] actions = {
			new Move(this, 100, 0, 0)
			
		};
		
		return actions;
	}
}

