package org.firstinspires.ftc.teamcode;


public class BotConfig {
  /*****************************************************************************
  ** HARDWARE CONSTANTS
  *****************************************************************************/
  public static final String FRONT_RIGHT_WHEEL_NAME = "front_right";
  public static final String FRONT_LEFT_WHEEL_NAME = "front_left";
  public static final String BACK_LEFT_WHEEL_NAME = "back_left";
  public static final String BACK_RIGHT_WHEEL_NAME = "back_right";
  
  public static final String ARM_LEFT_NAME = "left_arm";
  public static final String ARM_RIGHT_NAME = "right_arm";
  
  public static final String CLAW_LEFT_NAME = "claw_left";
  public static final String CLAW_RIGHT_NAME = "claw_right";
  public static final String WRIST_NAME = "wrist";
  
  public static final String DISTANCE_SENSOR_NAME = "dist_sensor";
  public static final String IMU_NAME = "imu";

  public static final String LEFT_ENCODER_NAME = "left_encoder";
  public static final String RIGHT_ENCODER_NAME = "right_encoder";
  public static final String HORIZONTAL_ENCODER_NAME = "horizontal_encoder";
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DISTANCE CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int TICKS_PER_360_DEG = 4150;
  public static final int TILE_LENGTH = 1250;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DRIVE SPEED CONSTANTS
  *****************************************************************************/
  public static final int CRUISE_SPEED = 2000;
  public static final double STRAFE_MULT = 1.1;
  public static final int ARM_VELOCITY = 1000;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ARM CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int BAR_HEIGHT = -450;
  public static final int BASKET_HEIGHT = 950;
  public static final int SPECIMEN_HEIGHT = 1490;
  // ---------------------------------------------------------------------------
  
  
  /*****************************************************************************
  ** CLAW CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final double CLAW_LEFT_OPEN_POS = 0.3;
	public static final double CLAW_LEFT_CLOSE_POS = 0.07;
	public static final double CLAW_RIGHT_OPEN_POS = 0.7;
	public static final double CLAW_RIGHT_CLOSE_POS = 0.93;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ODOMETRY CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final double FORWARD_OFFSET = 0; //TODO
  public static final double TRACK_WIDTH = 0; //TODO
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** CAMERA CONSTANTS
  *****************************************************************************/
  public static final String CAMERA_NAME = "Webcam 1";
	public static final int CAMERA_RESO_X = 640;
	public static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/teampiece.tflite";
	public static final String[] LABELS = { // Define the labels used in our model (must be in training order!)
		"BLUE",
		"RED",
	};
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** CAMERA CALIBRATION CONSTANTS
  * Lens intrinsics
  * UNITS ARE PIXELS
  * NOTE: this calibration is for the C920 webcam at 800x448.
  * You will need to do your own calibration for other configurations!

  Resolution: 1280x720
  Pixel Size: 2.8um
  Sensor Size: 3.58x2.02mm
  Stock lens focal length: 4.2mm
  *****************************************************************************/
  public static final double FX = 1430;
  public static final double FY = 1430;
  public static final double CX = 480;
  public static final double CY = 620;

  public static final double TAGSIZE = 0.166;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DETECTION CONSTANTS
  *****************************************************************************/
  public static final float DECIMATION_HIGH = 3;
  public static final float DECIMATION_LOW = 2;
  public static final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
  public static final int NUM_FRAMES_BEFORE_LOW_DECIMATION = 4;
  public static final int MAX_NUM_FRAMES_NO_DETECTION = 100; // How many attempts to detect before giving up
  // ---------------------------------------------------------------------------
}
