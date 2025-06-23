package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class MoveWristWithVelocity extends Action {
	Auto auto;
	double targetPosition;
	double velocity;
	boolean wait;
	double expectedTime = 0;
	ElapsedTime startTime;

	public MoveWristWithVelocity(Auto auto, double targetPosition, boolean wait, double velocity) {
		this.auto = auto;
		this.targetPosition = targetPosition;
		this.wait = wait;
		this.velocity = velocity;
	}
	
	public MoveWristWithVelocity(Auto auto, double targetPosition, boolean wait, double velocity, double expectedTime) {
		this.auto = auto;
		this.targetPosition = targetPosition;
		this.wait = wait;
		this.velocity = velocity;
		this.expectedTime = expectedTime;
	}

	public void onStart() {
		startTime = new ElapsedTime();
		this.auto.intake.MoveWrist(this.targetPosition);
		this.auto.intake.wrist.setVelocity((int)this.velocity);
	}

	public boolean isDone() {
		return !(this.wait && this.auto.intake.isWristBusy()) || ((expectedTime != 0) && (startTime.time() >= expectedTime));
	}
}
