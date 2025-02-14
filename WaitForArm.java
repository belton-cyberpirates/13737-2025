package org.firstinspires.ftc.teamcode;


public class WaitForArm extends Action {
	Auto auto;


	public WaitForArm(Auto auto) {
		this.auto = auto;
	}

	public void onStart() {}

	public boolean isDone() {
		return !this.auto.arm.isBusy();
	}
}