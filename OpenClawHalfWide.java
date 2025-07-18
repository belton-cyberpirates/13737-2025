package org.firstinspires.ftc.teamcode;


public class OpenClawHalfWide extends Action {
	Auto auto;

	public OpenClawHalfWide(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		auto.intake.SetClawPos(
			(BotConfig.CLAW_LEFT_FULL_OPEN_POS + BotConfig.CLAW_LEFT_OPEN_POS) / 2, 
			(BotConfig.CLAW_RIGHT_FULL_OPEN_POS + BotConfig.CLAW_RIGHT_OPEN_POS) / 2
		);
	}

	public boolean isDone() {
		return true;
	}
}
