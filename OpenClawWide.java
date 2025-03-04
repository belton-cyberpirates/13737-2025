package org.firstinspires.ftc.teamcode;


public class OpenClawWide extends Action {
	Auto auto;

	public OpenClawWide(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		auto.intake.SetClawPos(BotConfig.CLAW_LEFT_FULL_OPEN_POS, BotConfig.CLAW_RIGHT_FULL_OPEN_POS);
	}

	public boolean isDone() {
		return true;
	}
}
