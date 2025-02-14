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


	public void MoveWrist(int position) {
		wrist.setVelocity(BotConfig.WRIST_VELOCITY);
		wrist.setTargetPosition(position);
	}
  
  
	public void OpenClaw() {
		claw_left.setPosition(BotConfig.CLAW_LEFT_OPEN_POS);
		claw_right.setPosition(BotConfig.CLAW_RIGHT_OPEN_POS);
	}
  
  
	public void CloseClaw() {
		claw_left.setPosition(BotConfig.CLAW_LEFT_CLOSE_POS);
		claw_right.setPosition(BotConfig.CLAW_RIGHT_CLOSE_POS);
	}


	public boolean isWristBusy() {
		return wrist.isBusy();
	}
}