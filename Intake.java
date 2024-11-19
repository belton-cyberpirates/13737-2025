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
	private Servo clawLeft;
	private Servo clawRight;
	

	public Intake(LinearOpMode auto) {
		this.auto = auto;
		this.wrist = auto.hardwareMap.get(Servo.class, BotConfig.WRIST_NAME);
		this.clawLeft = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_LEFT_NAME);
		this.clawRight = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_RIGHT_NAME);
	}


	public void MoveWrist(double position) {
		MoveWrist(position, 0);
	}
	

	public void MoveWrist(double position, int wait) {
		wrist.setPosition(position);
		auto.sleep(wait);
	}
	
  
	public void OpenClaw(int wait) {
		clawLeft.setPosition(BotConfig.CLAW_LEFT_OPEN);
		clawRight.setPosition(BotConfig.CLAW_RIGHT_OPEN);
		auto.sleep(wait);
	}
  
  
	public void OpenClaw() {
		OpenClaw(0);
	}
  
  
	public void CloseClaw(int wait) {
		clawLeft.setPosition(BotConfig.CLAW_LEFT_CLOSE);
		clawRight.setPosition(BotConfig.CLAW_RIGHT_CLOSE);
		auto.sleep(wait);
	}
  
  
	public void CloseClaw() {
		CloseClaw(0);
	}
}