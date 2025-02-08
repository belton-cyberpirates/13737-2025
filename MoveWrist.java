package org.firstinspires.ftc.teamcode;


public class MoveWrist extends Action {
    Auto auto;
    int targetPosition;
    boolean wait;


    public MoveWrist(Auto auto, int targetPosition, boolean wait) {
        this.auto = auto;
        this.targetPosition = targetPosition;
        this.wait = wait;
    }

    public void onStart() {
        this.auto.intake.MoveWrist(this.targetPosition);
    }

    public boolean isDone() {
        return !(this.wait && this.auto.intake.isWristBusy());
    }
}