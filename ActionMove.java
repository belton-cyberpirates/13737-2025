package org.firstinspires.ftc.teamcode;


public class ActionMove extends Action {
	Auto auto;
	double xPos;
	double yPos;
	double heading;

	public ActionMove(Auto auto, double xPos, double yPos, double heading) {
		this.auto = auto;
		
		this.xPos = xPos;
		this.yPos = yPos;
		this.heading = heading;
	}

	public void onStart() {
		auto.driveMotors.Move(xPos, yPos, heading);
	}
	
	public boolean isDone() {
		return auto.driveMotors.isDone();
	}

}