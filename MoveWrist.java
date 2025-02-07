package org.firstinspires.ftc.teamcode;


public class MoveWrist extends Action {
    Auto auto;
    int targetPosition;
    boolean wait;


    public MoveArm(Auto auto, int targetPosition, boolean wait) {
        this.auto = auto;
        this.targetPosition = position;
        this.wait = wait;
    }

    public onStart() {
        this.auto.intake.MoveWrist(this.targetPosition);
    }

    public isDone() {
        return !(this.wait && this.auto.intake.isWristBusy());
    }
}