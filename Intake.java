package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BotConfig;
import java.util.List;

public class Intake {
	private LinearOpMode auto;
	private Servo wrist;
	private Servo claw;
	

	public Intake(LinearOpMode auto) {
		this.auto = auto;
		this.wrist = auto.hardwareMap.get(Servo.class, BotConfig.WRIST_NAME);
		this.claw = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_NAME);
	}


	public void MoveWrist(double position) {
		MoveWrist(position, 0);
	}
	

	public void MoveWrist(double position, int wait) {
		wrist.setPosition(position);
		auto.sleep(wait);
	}
	
  
	public void OpenClaw(int wait) {
		claw.setPosition(BotConfig.CLAW_OPEN);
		auto.sleep(wait);
	}
  
  
	public void OpenClaw() {
		OpenClaw(0);
	}
  
  
	public void CloseClaw(int wait) {
		claw.setPosition(BotConfig.CLAW_CLOSE);
		auto.sleep(wait);
	}
  
  
	public void CloseClaw() {
		CloseClaw(0);
	}
}