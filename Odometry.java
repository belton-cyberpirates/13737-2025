package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Odometry {
	private LinearOpMode auto;

	private DcMotorEx encoderLeft;
	private DcMotorEx encoderRight;
	private DcMotorEx encoderHorizontal;

	double xPos = 0;
	double yPos = 0;
	double heading = 0;

	double prevLeftPos = 0;
	double prevRightPos = 0;
	double prevHorizontalPos = 0;


	public Odometry(LinearOpMode auto) {
		this.auto = auto;

		this.encoderLeft = auto.hardwareMap.get(DcMotorEx.class, BotConfig.LEFT_ENCODER_NAME);
		this.encoderRight = auto.hardwareMap.get(DcMotorEx.class, BotConfig.RIGHT_ENCODER_NAME);
		this.encoderHorizontal = auto.hardwareMap.get(DcMotorEx.class, BotConfig.HORIZONTAL_ENCODER_NAME);
	}


	public void process() {
		double leftPos = -this.encoderLeft.getCurrentPosition() / BotConfig.TICKS_PER_MM;
		double rightPos = this.encoderRight.getCurrentPosition() / BotConfig.TICKS_PER_MM;
		double horizontalPos = this.encoderHorizontal.getCurrentPosition() / BotConfig.TICKS_PER_MM;

		double deltaLeft = leftPos - prevLeftPos;
		double deltaRight = rightPos - prevRightPos;
		double deltaHorizontal = horizontalPos - prevHorizontalPos;
		
		this.prevLeftPos = leftPos;
		this.prevRightPos = rightPos;
		this.prevHorizontalPos = horizontalPos;

		updatePosition(deltaLeft, deltaRight, deltaHorizontal);
	}


	void updatePosition(double deltaLeft, double deltaRight, double deltaHorizontal) {
		double deltaHeading = (deltaLeft - deltaRight) / BotConfig.TRACK_WIDTH;

		double centerDisplacement = (deltaLeft + deltaRight) / 2;
		double horizontalDisplacement = deltaHorizontal - (BotConfig.FORWARD_OFFSET * deltaHeading);

		double deltaY = (
				centerDisplacement * ( Math.sin(heading + deltaHeading) - Math.sin(heading) ) + 
				horizontalDisplacement * ( Math.cos(heading + deltaHeading) - Math.cos(heading) ) 
			) / deltaHeading;
		double deltaX = ( 
				horizontalDisplacement * ( Math.sin(heading + deltaHeading) - Math.sin(heading) ) - 
				centerDisplacement * ( Math.cos(heading + deltaHeading) - Math.cos(heading) ) 
			) / deltaHeading;
		
		if (!( Double.isNaN(deltaX) || Double.isNaN(deltaY) )) {
			xPos += deltaX;
			yPos += deltaY;
			heading += deltaHeading;
		}
		
		auto.telemetry.addData("odometry deltaLeft", deltaLeft);
		auto.telemetry.addData("odometry deltaRight", deltaRight);
		auto.telemetry.addData("odometry deltaHorizontal", deltaHorizontal);
		auto.telemetry.addLine();
		auto.telemetry.addData("odometry deltaHeading", deltaHeading);
		auto.telemetry.addData("odometry centerDisplacement", centerDisplacement);
		auto.telemetry.addData("odometry horizontalDisplacement", horizontalDisplacement);
		auto.telemetry.addData("odometry deltaX", deltaX);
		auto.telemetry.addData("odometry deltaY", deltaY);
		auto.telemetry.addLine();
		auto.telemetry.addData("odometry pos", xPos + ", " + yPos);
		auto.telemetry.addData("odometry heading", heading);
		auto.telemetry.addLine();
	}


	public double getX() {
		return xPos;
	}

	public double getY() {
		return yPos;
	}

	public double getHeading() {
		return heading;
	}
}