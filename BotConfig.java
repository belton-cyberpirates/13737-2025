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

  public static final String LEFT_ENCODER_NAME = "back_right";
  public static final String RIGHT_ENCODER_NAME = "back_left";
  public static final String HORIZONTAL_ENCODER_NAME = "front_right";
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DISTANCE CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int TILE_LENGTH = 595; // MM
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** DRIVE SPEED CONSTANTS
  *****************************************************************************/
  public static final double STRAFE_MULT = 1.41;
  public static final int ARM_VELOCITY = 2000;
  public static final int WRIST_VELOCITY = 1500;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** BASE CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int BAR_X = 665;
  public static final int BAR_SCORE_X = BAR_X - 350;
  
  public static final int PICKUP_X = 390;
  public static final int PICKUP_Y = -430;
  
  public static final int BLOCK_GRAB_X = 320;
  
  public static final int PLOW_X = 1200;
  
  public static final int BASKET_X = 300;
  public static final int BASKET_Y = 1000;
  
  public static final int FIRST_BLOCK_Y = -1000;
  public static final int SECOND_BLOCK_Y = -1240;
  
  public static final int THIRD_BLOCK_X = 880;
  public static final int THIRD_BLOCK_Y = -980;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ARM CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final double BAR_HEIGHT = 0;
  public static final double BASKET_HEIGHT = 600;
  public static final double BASKET_SAFE_HEIGHT = 490;
  public static final double MAX_ARM_HEIGHT = 600;
  // ---------------------------------------------------------------------------
  
  
  /*****************************************************************************
  ** WRIST CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final int WRIST_SPECIMEN_HEIGHT = 1370;
  public static final int WRIST_SAMPLE_HEIGHT = 1405;
  public static final int WRIST_SIDE_SAMPLE_HEIGHT = 1425;
  
  public static final int WRIST_BAR_HEIGHT = 600;
  public static final int WRIST_PASSIVE = 500;
  public static final int WRIST_DUNK_HEIGHT = 760;
  public static final int WRIST_BASKET_SAFE_HEIGHT = 670;
  
  // ---------------------------------------------------------------------------
  
  
  /*****************************************************************************
  ** CLAW CALIBRATION CONSTANTS
  *****************************************************************************/
	public static final double CLAW_LEFT_OPEN_POS = 0.2;
	public static final double CLAW_LEFT_FULL_OPEN_POS = 0.4;
	public static final double CLAW_LEFT_CLOSE_POS = 0.07;
	public static final double CLAW_LEFT_HALF_CLOSE_POS = (CLAW_LEFT_OPEN_POS + CLAW_LEFT_CLOSE_POS) / 2;

	public static final double CLAW_RIGHT_OPEN_POS = 0.8;
	public static final double CLAW_RIGHT_FULL_OPEN_POS = 0.6;
	public static final double CLAW_RIGHT_CLOSE_POS = 0.93;
	public static final double CLAW_RIGHT_HALF_CLOSE_POS = (CLAW_RIGHT_OPEN_POS + CLAW_RIGHT_CLOSE_POS) / 2;
	
	public static final int HUMAN_WAIT_TIME = 500;
  // ---------------------------------------------------------------------------


  /*****************************************************************************
  ** ODOMETRY CALIBRATION CONSTANTS
  *****************************************************************************/
  public static final double FORWARD_OFFSET = 16.25;
  public static final double TRACK_WIDTH = 367;
  public static final double WHEEL_DIAMETER = 38;
  public static final double TICKS_PER_REVOLUTION = 2048;
  public static final double TICKS_PER_MM = (int)( TICKS_PER_REVOLUTION / ( Math.PI * WHEEL_DIAMETER ) );
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
