package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Direction;


public class Arm {
	private LinearOpMode auto;
	private DcMotorEx leftArm;
	private DcMotorEx rightArm;
	private DcMotorEx[] motors;


	public Arm(LinearOpMode auto) {
		this.auto = auto;
		this.leftArm = auto.hardwareMap.get(DcMotorEx.class, BotConfig.ARM_LEFT_NAME);
		this.rightArm = auto.hardwareMap.get(DcMotorEx.class, BotConfig.ARM_RIGHT_NAME);

		// create list of motors to make code cleaner
		motors = new DcMotorEx[]{ this.leftArm, this.rightArm };
  	}

  
	public void DropArm() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftArm.setPower(.2);
		rightArm.setPower(.2);
	}

  
	public void Initialize() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		for(DcMotorEx motor : motors) motor.setTargetPosition(50);
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftArm.setPower(1);
		rightArm.setPower(1);
	}
  
	private void setVelocity(int armVelocity) {
		for(DcMotorEx motor : motors) motor.setVelocity(armVelocity);
	}
  
	public void Move(int position) {
		setVelocity(BotConfig.ARM_VELOCITY);

		leftArm.setTargetPosition(-position);
		rightArm.setTargetPosition(-position);
	}
	
	public boolean isBusy() {
		return leftArm.isBusy() || rightArm.isBusy();
	}
}