package org.firstinspires.ftc.teamcode;


public class OpenClaw extends Action {
    Auto auto;

    public OpenClaw(Auto auto) {
        this.auto = auto;
    }

    public void onStart() {
        auto.intake.OpenClaw();
    }

    public boolean isDone() {
        return true;
    }
}
