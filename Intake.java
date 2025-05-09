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
	public DcMotorEx wrist;
	private Servo claw_left;
	private Servo claw_right;
	
	private Servo differential_left;
	private Servo differential_right;

	states state;
	

	public Intake(LinearOpMode auto) {
		this.auto = auto;
		
		this.wrist = auto.hardwareMap.get(DcMotorEx.class, BotConfig.WRIST_NAME);
		
		this.wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		this.claw_left = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_LEFT_NAME);
		this.claw_right = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_RIGHT_NAME);
		
		this.differential_left = auto.hardwareMap.get(Servo.class, BotConfig.DIFFERENTIAL_LEFT_NAME);
		this.differential_right = auto.hardwareMap.get(Servo.class, BotConfig.DIFFERENTIAL_RIGHT_NAME);
		
		wrist.setTargetPosition(0);
		
	}
	
	
	public void DropWrist() {
		wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		wrist.setPower(-.05);
	}
	
	
	public void InitializeWrist() {
		wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		wrist.setTargetPosition(0);
		wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		wrist.setPower(1);
	}
	
	
	public void MoveDifferential(double pitch, double roll) {
		
	  double angle1 = pitch / BotConfig.DIFFERENTIAL_SERVO_DEGREES;
	  double angle2 = roll / BotConfig.DIFFERENTIAL_SERVO_DEGREES / 2;
	  
	  differential_left.setPosition(0.5 + angle1 + angle2);
	  differential_right.setPosition(0.5 - angle1 + angle2);
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

		//if (wrist.getTargetPosition() == targetPosition) { return; }

		wrist.setTargetPosition((int)targetPosition);
	}


	public void setState(states newState) {
		if (this.state == newState) { return; }
		this.state = newState;

		switch (newState) {
			case POWER:
				wrist.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				break;
			
			case VELOCITY:
				wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				break;
			
			case POSITION:
				wrist.setVelocity(BotConfig.WRIST_VELOCITY);
				wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				break;
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
	
	public int getWristPos() {
		return wrist.getCurrentPosition();
	}
}