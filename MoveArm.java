package org.firstinspires.ftc.teamcode;


public class MoveArm extends Action {
    Auto auto;
    int targetPosition;
    boolean wait;


    public MoveArm(Auto auto, int targetPosition, boolean wait) {
        this.auto = auto;
        this.targetPosition = targetPosition;
        this.wait = wait;
    }

    public void onStart() {
        this.auto.arm.Move(this.targetPosition);
    }

    public boolean isDone() {
        return !(this.wait && this.auto.arm.isBusy());
    }
}