package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Direction;


public class Arm {

	enum states {
		POWER,
		VELOCITY,
		POSITION
	}

	private LinearOpMode auto;

	private DcMotorEx leftArm;
	private DcMotorEx rightArm;

	private DcMotorEx[] motors;

	public states state;


	public Arm(LinearOpMode auto) {
		this.auto = auto;

		this.leftArm = auto.hardwareMap.get(DcMotorEx.class, BotConfig.ARM_LEFT_NAME);
		this.rightArm = auto.hardwareMap.get(DcMotorEx.class, BotConfig.ARM_RIGHT_NAME);
		
		this.leftArm.setTargetPosition(0);
		this.rightArm.setTargetPosition(0);

		// create list of motors to make code cleaner
		motors = new DcMotorEx[]{ this.leftArm, this.rightArm };
	}


	public void DropArm() {
		MoveWithPower(-.1);
	}

  	
	public void process() {

	}


	public void MoveWithPower(double power) {
		setState(states.POWER);

		leftArm.setPower(power);
		rightArm.setPower(-power);
	}


	public void MoveWithVelocity(double velocity) {
		setState(states.VELOCITY);

		leftArm.setVelocity(-velocity);
		rightArm.setVelocity(velocity);
	}


	public void Move(double targetPosition) {
		setState(states.POSITION);

		if (leftArm.getTargetPosition() == targetPosition) { return; }

		leftArm.setTargetPosition(-(int)targetPosition);
		rightArm.setTargetPosition((int)targetPosition);
	}


	public void setState(states newState) {
		if (this.state == newState) { return; }
		this.state = newState;

		switch (newState) {
			case POWER:
				for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				break;
			
			case VELOCITY:
				for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				break;
			
			case POSITION:
				setVelocity(BotConfig.ARM_VELOCITY);
				for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				break;
		}
	}

  
	public void Initialize() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
  
	private void setVelocity(int armVelocity) {
		for(DcMotorEx motor : motors) motor.setVelocity(armVelocity);
	}
	
	public boolean isBusy() {
		return leftArm.isBusy() || rightArm.isBusy();
	}
	
	public int getHeight() {
		return rightArm.getCurrentPosition();
	}
}