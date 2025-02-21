package org.firstinspires.ftc.teamcode;


public class CloseClaw extends Action {
	Auto auto;

	public CloseClaw(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {
		this.auto.intake.CloseClaw();
	}

	public boolean isDone() {
		return true;
	}
}
