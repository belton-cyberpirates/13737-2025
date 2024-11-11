package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.List;

import org.firstinspires.ftc.teamcode.DriveMotors;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.BotConfig;
import org.firstinspires.ftc.teamcode.Auto;


@Autonomous(name = "Left", preselectTeleOp="Field Centric (main)")
public class AutoLeft extends Auto {
	private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
	private AprilTagProcessor aprilTag;
	private VisionPortal visionPortal;
	
	@Override
	public void runOpMode() {
		Initialize();
		
		initAprilTag();

		waitForStart();

		if (opModeIsActive()) { // <----------------------------------------------------------------
				//sleep(20000);
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * 1));
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 2.1));
	  		//begin to push the blocks
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * .5));
	  		driveMotors.Move(Direction.BACKWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * .5));
	  		driveMotors.Move(Direction.BACKWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		driveMotors.Move(Direction.LEFT, (int)(BotConfig.TILE_LENGTH * .5));
	  		driveMotors.Move(Direction.BACKWARD, (int)(BotConfig.TILE_LENGTH * 1.9));
	  		driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 2));
	  		// Rotate towards apriltag
	  		driveMotors.Turn(-90);
	  		// move until apriltag detected
	  		// List<AprilTagDetection> currentDetections = aprilTag.getDetections();
	  		// while (currentDetections.size() <= 0) {
	  		// 	driveMotors.SetVelocity(1);
	  		// }
	  		//park/low ascent 
	  		//driveMotors.Turn(90);
	  		//driveMotors.Move(Direction.FORWARD, (int)(BotConfig.TILE_LENGTH * 1.7));
		}
		saveHeading();
	}
	
	private void initAprilTag() {

		// Create the AprilTag processor.
		aprilTag = new AprilTagProcessor.Builder()

			// The following default settings are available to un-comment and edit as needed.
			//.setDrawAxes(false)
			//.setDrawCubeProjection(false)
			//.setDrawTagOutline(true)
			//.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
			//.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
			//.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

			// == CAMERA CALIBRATION ==
			// If you do not manually specify calibration parameters, the SDK will attempt
			// to load a predefined calibration for your camera.
			//.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
			// ... these parameters are fx, fy, cx, cy.

			.build();

		// Adjust Image Decimation to trade-off detection-range for detection-rate.
		// eg: Some typical detection data using a Logitech C920 WebCam
		// Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
		// Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
		// Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
		// Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
		// Note: Decimation can be changed on-the-fly to adapt during a match.
		//aprilTag.setDecimation(3);

		// Create the vision portal by using a builder.
		VisionPortal.Builder builder = new VisionPortal.Builder();

		// Set the camera (webcam vs. built-in RC phone camera).
		if (USE_WEBCAM) {
			builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
		} else {
			builder.setCamera(BuiltinCameraDirection.BACK);
		}

		// Choose a camera resolution. Not all cameras support all resolutions.
		//builder.setCameraResolution(new Size(640, 480));

		// Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
		//builder.enableLiveView(true);

		// Set the stream format; MJPEG uses less bandwidth than default YUY2.
		//builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

		// Choose whether or not LiveView stops if no processors are enabled.
		// If set "true", monitor shows solid orange screen if no processors enabled.
		// If set "false", monitor shows camera view without annotations.
		//builder.setAutoStopLiveView(false);

		// Set and enable the processor.
		builder.addProcessor(aprilTag);

		// Build the Vision Portal, using the above settings.
		visionPortal = builder.build();

		// Disable or re-enable the aprilTag processor at any time.
		//visionPortal.setProcessorEnabled(aprilTag, true);

	}   // end method initAprilTag()
}
