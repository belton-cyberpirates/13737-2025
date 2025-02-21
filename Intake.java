package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BotConfig;
import java.util.List;

public class Intake {

	enum states {
		POWER,
		VELOCITY,
		POSITION
	}

	private LinearOpMode auto;
	private DcMotorEx wrist;
	private Servo claw_left;
	private Servo claw_right;

	states state;
	

	public Intake(LinearOpMode auto) {
		this.auto = auto;
		this.wrist = auto.hardwareMap.get(DcMotorEx.class, BotConfig.WRIST_NAME);
		this.claw_left = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_LEFT_NAME);
		this.claw_right = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_RIGHT_NAME);
		
		wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		wrist.setTargetPosition(0);
		wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		wrist.setPower(1);
	}
	
	
	public void DropWrist() {
		wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		wrist.setPower(-.2);
	}
	
	
	public void InitializeWrist() {
		wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		wrist.setTargetPosition(0);
		wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		wrist.setPower(1);
	}


	public void MoveWristWithPower(double power) {
		setState(states.POWER);

		wrist.setPower(power);
	}


	public void MoveWristWithVelocity(double velocity) {
		setState(states.VELOCITY);

		wrist.setVelocity(velocity);
	}


	public void MoveWrist(double targetPosition) {
		setState(states.POSITION);

		if (wrist.targetPosition == targetPosition) { return; }

		wrist.setTargetPosition(targetPosition);
	}


	public void setState(states newState) {
		if (this.state == newState) { return; }

		switch (newState) {
			case states.POWER:
				wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			
			case states.VELOCITY:
				wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			
			case states.POSITION:
				setVelocity(BotConfig.WRIST_VELOCITY);
				wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
	}
  
  
	public void OpenClaw() {
		SetClawPos(BotConfig.CLAW_LEFT_OPEN_POS, BotConfig.CLAW_RIGHT_OPEN_POS);
	}
  
  
	public void CloseClaw() {
		SetClawPos(BotConfig.CLAW_LEFT_CLOSE_POS, BotConfig.CLAW_RIGHT_CLOSE_POS);
	}


	public void SetClawPos(double leftPos, double rightPos) {
		claw_left.setPosition(leftPos);
		claw_right.setPosition(rightPos);
	}


	public boolean isWristBusy() {
		return wrist.isBusy();
	}
}