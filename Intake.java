package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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
        this.claw = auto.hardwareMap.get(Servo.class, BotConfig.CLAW_NAME);
    }


    public void MoveWrist(double position) {
        MoveWrist(position, 0);
    }
    

    public void MoveWrist(double position, int wait) {
        wrist.setPosition(position);
        auto.sleep(wait);
    }
    
  
    private void OpenClaw(int wait) {
        claw.setPosition(BotConfig.CLAW_OPEN);
        auto.sleep(wait);
    }
  
  
    private void OpenClaw() {
        OpenClaw(0);
    }
  
  
    private void CloseClaw(int wait) {
        claw.setPosition(BotConfig.CLAW_CLOSE);
        auto.sleep(wait);
    }
  
  
    private void CloseClaw() {
        CloseClaw(0);
    }
}