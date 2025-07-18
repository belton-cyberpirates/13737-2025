package org.firstinspires.ftc.teamcode;


public class WaitForWrist extends Action {
	Auto auto;


	public WaitForWrist(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {}

	public boolean isDone() {
		return !this.auto.intake.isWristBusy();
	}
}
