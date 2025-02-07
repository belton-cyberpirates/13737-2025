package org.firstinspires.ftc.teamcode;


public class MoveArm extends Action {
    Auto auto;
    int targetPosition;
    boolean wait;


    public MoveArm(Auto auto, int targetPosition, boolean wait) {
        this.auto = auto;
        this.targetPosition = position;
        this.wait = wait;
    }

    public onStart() {
        this.auto.arm.Move(this.targetPosition);
    }

    public isDone() {
        return !(this.wait && this.auto.arm.isBusy());
    }
}