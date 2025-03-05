package org.firstinspires.ftc.teamcode;


public class MoveArm extends Action {
	Auto auto;
	double targetPosition;
	boolean wait;
	double expectedTime = 0;
	ElapsedTime startTime;


	public MoveArm(Auto auto, double targetPosition, boolean wait) {
		this.auto = auto;
		this.targetPosition = targetPosition;
		this.wait = wait;
	}

	public MoveArm(Auto auto, double targetPosition, boolean wait, double expectedTime) {
		this.auto = auto;
		this.targetPosition = targetPosition;
		this.wait = wait;
		this.expectedTime = expectedTime;
	}

	public void onStart() {
		startTime = new ElapsedTime();
		this.auto.arm.Move(this.targetPosition);
	}

	public boolean isDone() {
		return !(this.wait && this.auto.arm.isBusy()) || (expectedTime != 0 && startTime.time() >= expectedTime);;
	}
}
