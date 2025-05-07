package org.firstinspires.ftc.teamcode;


public class MoveDifferential extends Action {
	Auto auto;
	Double pitch;
	Double roll;

	public MoveDifferential(Auto auto, double pitch, double roll) {
		this.auto = auto;
		this.pitch = pitch;
		this.roll = roll;
	}

	public void onStart() {
		auto.intake.MoveDifferential(pitch, roll);
	}

	public boolean isDone() {
		return true;
	}
}
