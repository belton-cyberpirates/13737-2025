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
	private LinearOpMode auto;
	private DcMotorEx leftArm;
	private DcMotorEx rightArm;
	private DcMotorEx[] motors;
	
	private PIDController pidController = new PIDController(1.3, 0.06, 0.2);
	private double targetPosition;
	private AnalogInput armPot;
	private ElapsedTime deltaTimer = new ElapsedTime();
	private ElapsedTime armTimer = new ElapsedTime();


	public Arm(LinearOpMode auto) {
		this.auto = auto;
		this.leftArm = auto.hardwareMap.get(DcMotorEx.class, BotConfig.ARM_LEFT_NAME);
		this.rightArm = auto.hardwareMap.get(DcMotorEx.class, BotConfig.ARM_RIGHT_NAME);
		this.armPot = auto.hardwareMap.get(AnalogInput.class, "arm_pot");

		// create list of motors to make code cleaner
		motors = new DcMotorEx[]{ this.leftArm, this.rightArm };
	}
  	
	public void process() {
		double deltaTime = deltaTimer.seconds();
		
		double currHeight = armPot.getVoltage();
		double error = targetPosition - currHeight;
		
		double power = pidController.PIDControl(error, deltaTime);

		leftArm.setPower(power);
		rightArm.setPower(power);
		
		auto.telemetry.addData("arm error", pidController.lastOutput);
		auto.telemetry.addData("arm power", power);
		auto.telemetry.addData("arm value", armPot.getVoltage());
		auto.telemetry.addData("arm error", error);
		
		deltaTimer.reset();
	}

  
	public void Initialize() {
		for(DcMotorEx motor : motors) motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
  
	private void setVelocity(int armVelocity) {
		for(DcMotorEx motor : motors) motor.setVelocity(armVelocity);
	}
  
	public void Move(double targetPosition) {
		armTimer.reset();
		this.targetPosition = targetPosition;
	}
	
	public boolean isBusy() {
		return armTimer.milliseconds() < 500 ||
			(Math.abs(pidController.lastError) > .05);
	}
}