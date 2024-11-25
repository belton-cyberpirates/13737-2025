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
	private LinearOpMode auto;
	private DcMotorEx wrist;
	private Servo claw_left;
	private Servo claw_right;
	

	public Intake(LinearOpMode auto) {
		this.auto = auto;
		this.wrist = auto.hardwareMap.get(DcMotorEx.class, BotConfig.WRIST_NAME);
		this.claw_left = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_LEFT_NAME);
		this.claw_right = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_RIGHT_NAME);
		
		wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}


	public void MoveWrist(int position) {
		MoveWrist(position, 0);
	}
	

	public void MoveWrist(int position, int wait) {
		wrist.setTargetPosition(position);
		auto.sleep(wait);
	}
	

	public void MoveWrist(int position, boolean wait) {
		wrist.setTargetPosition(position);
		if (wait) {
			while (wrist.isBusy()) {}
		}
	}
	
  
	public void OpenClaw(int wait) {
		claw_left.setPosition(BotConfig.CLAW_LEFT_OPEN_POS);
		claw_right.setPosition(BotConfig.CLAW_RIGHT_OPEN_POS);
		auto.sleep(wait);
	}
  
  
	public void OpenClaw() {
		OpenClaw(0);
	}
  
  
	public void CloseClaw(int wait) {
		claw_left.setPosition(BotConfig.CLAW_LEFT_CLOSE_POS);
		claw_right.setPosition(BotConfig.CLAW_RIGHT_CLOSE_POS);
		auto.sleep(wait);
	}
  
  
	public void CloseClaw() {
		CloseClaw(0);
	}
}