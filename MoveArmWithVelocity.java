package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class MoveArmWithVelocity extends Action {
	Auto auto;
	double targetPosition;
	double velocity;
	boolean wait;
	double expectedTime = 0;
	ElapsedTime startTime;

	public MoveArmWithVelocity(Auto auto, double targetPosition, boolean wait, double velocity) {
		this.auto = auto;
		this.targetPosition = targetPosition;
		this.wait = wait;
		this.velocity = velocity;
	}
	
	public MoveArmWithVelocity(Auto auto, double targetPosition, boolean wait, double velocity, double expectedTime) {
		this.auto = auto;
		this.targetPosition = targetPosition;
		this.wait = wait;
		this.velocity = velocity;
		this.expectedTime = expectedTime;
	}

	public void onStart() {
		startTime = new ElapsedTime();
		this.auto.arm.Move(this.targetPosition);
		this.auto.arm.setVelocity(this.velocity);
	}

	public boolean isDone() {
		return !(this.wait && this.auto.arm.isBusy()) || (expectedTime != 0 && startTime.time() >= expectedTime);
	}
}
