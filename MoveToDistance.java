package org.firstinspires.ftc.teamcode;


public class MoveToDistance extends Action {
	Auto auto;
	int targetPosition;


	public MoveToDistance(Auto auto, int targetPosition) {
		this.auto = auto;
		this.targetPosition = targetPosition;
	}

	public void onStart() {
		this.auto.driveMotors.MoveToDistance(targetPosition);
	}

	public boolean isDone() {
		return !(auto.driveMotors.distanceSensorPidController.lastError < .05);
	}
}