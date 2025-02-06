package org.firstinspires.ftc.teamcode;


public class Odometry {
    private LinearOpMode auto;

    private Encoder encoderLeft;
    private Encoder encoderRight;
    private Encoder encoderHorizontal;

    double xPos = 0;
    double yPos = 0;
    double heading = 0;


    public Odometry(LinearOpMode auto) {
        this.auto = auto;

        this.encoderLeft = auto.hardwareMap.get(Encoder.class, BotConfig.LEFT_ENCODER_NAME);
        this.encoderRight = auto.hardwareMap.get(Encoder.class, BotConfig.RIGHT_ENCODER_NAME);
        this.encoderHorizontal = auto.hardwareMap.get(Encoder.class, BotConfig.HORIZONTAL_ENCODER_NAME);

        new Thread(this::updateLoop).start();
    }


    void updatePosition(double deltaLeft, double deltaRight, double deltaHorizontal) {
        deltaHeading = (deltaLeft - deltaRight) / BotConfig.TRACK_WIDTH;

        double centerDisplacement = (deltaLeft + deltaRight) / 2
        double horizontalDisplacement = deltaHorizontal - (BotConfig.FORWARD_OFFSET * deltaHeading)
    }


    void updateLoop() {
        while (auto.opModeIsActive()) {
            double deltaLeft = this.encoderLeft.getPosition();
            double deltaRight = this.encoderRight.getPosition();
            double deltaHorizontal = this.encoderHorizontal.getPosition();

            updatePosition(deltaLeft, deltaRight, deltaHorizontal)
        }
    }


    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }

    public double getHeading() {
        return heading;
    }
}