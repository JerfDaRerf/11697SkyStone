/*
https://github.com/Rambotics/FTC-2016-2017-v2.4-pc/tree/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode
https://github.com/pmtischler/ftc_app/tree/master/SharedCode/src/main/java/com/github/pmtischler

Code Folding Expand/Collapse ALL => Control || Shift || +/-
Last Edit Location => Control + Shift + BackSpace
Add Bookmark => Control + F11 + number
Find Bookmark => Control + number
Show Bookmarks => Shift + F11
Jump to Declaration => Control + B


*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

import static java.lang.Thread.sleep;

@Autonomous(name="BlueStoned", group ="11697")
//BLUE Left has "public class BlueLeft extends LinearOpMode
public class BlueStoned extends LinearOpMode {
    Hardware robot = new Hardware();
    /**
     * PROGRAM CONFIGURATION VARIABLES
     */
    private static boolean closeNavWall = false;
    private static boolean closeSkystone = false;
    private static boolean dragFoundation = false;
    private static boolean closeParkWall = false;

    // Motors Settings [DO NOT CHANGE]
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double DRIVE_GEAR_REDUCTION = 1.3;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Used by system functions
    private ElapsedTime moveTimer = new ElapsedTime();
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    private String PREVIOUS_MSG = "";
    private static String stonePosition = "";

    /*
       Vuforia License Key can be found at:
       https://developer.vuforia.com/license-manager
     */

    private static final String VUFORIA_KEY = "ARYNigX/////AAABmeXRDofbC0DPuqRvnFl7Lm0xLd3BcZs7CL0R30y56aNmiqgHfjjzoNz1tmjW/mCUTlFCDlebwZwBvbsUYHVnFbvZDj4JAMqCWXjfeQGIPZLKg5ySTrg49Bx0UiaJybDMZhiJfdPAafR02xAXGXvnA8Uoa7/jomwlc2djsoG08Z2b5hKK83LT3nsWixPjixBedVs33VFKTF8w1oarxQwJEUf4GRvBeQVXqz7wgLIeMS1j8NKkcfDLQP1fPTPZINmkSI6vEutYGzPEGl827w37aUx6eszeaw/TEMVWeQFxIZdCwa8uEgPRph695Z1yJu4UoLy6mQXZMXMFTxmW+Z6SCKWIqmNPKLGnRtSNzQol2V9y";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //ADD VALUES FOR SKYSTONE DETECTION
    private int stoneX = -1;
    private int skystoneX = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        sleep(1500);

        //Preliminary skystone scan
        stoneDetect(tfod);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // Calibrate IMU before continuing
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        robot.pinchyBoi.setPosition(0.12);

        waitForStart();


        while (opModeIsActive()) {
            stoneDetect(tfod);
            correction = checkDirection();

            sleep(500);

            driveByEncoder(0.4, -3, -3);

            //Move toward stones
            driveByEncoder(0.4, -2.7, -2.7);

            robot.frontFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontFlip.setTargetPosition(robot.frontFlip.getCurrentPosition() - 50);
            robot.frontFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontFlip.setPower(0.3);

            robot.pinchyBoi.setPosition(0.0);

            sleep(500);

            robot.frontFlip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            switch (stonePosition) {
                case "LEFT":
                    //Strafe to center
                    driveByEncoder(0.15, 1.85, 0);

                    //Move towards SS
                    driveByEncoder(0.3,-3.5,-3.5);

                    driveByEncoder(0.15, -0.7, -0.7);

                    //Grab skystone
                    robot.pinchyBoi.setPosition(0.6);

                    sleep(500);

                    robot.verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() + 550);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.6);

                    sleep(200);

                    //Move away from SS
                    driveByEncoder(0.3, 3, 3);

                    //Turn to face foundation
                    rotate(85, 0.35);

                    driveByEncoder(0.15, 2.5,0);

                    //Drive to foundation
                    driveByEncoder(0.7,-31.5,-31.5);

                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() + 525);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.55);

                    //Turn to face foundation
                    rotate(-85, 0.3);

                    //Drive up to foundation
                    driveByEncoder(0.3,-5.5,-5.5);

                    sleep(200);

                    robot.pinchyBoi.setPosition(0.0);

                    robot.clampyBoi1.setPosition(0.47);
                    robot.clampyBoi2.setPosition(0.52);
                    sleep(200);

                    //Drive to building zone
                    driveByEncoder(0.3,15,15);

                    //Lift clamping arms
                    robot.clampyBoi1.setPosition(0.66);
                    robot.clampyBoi2.setPosition(0.19);

                    driveByEncoder(0.3, 28,0);

                    //driveByEncoder(0.3,14,0);
                    break;

                case "CENTER":
                    //Strafe to center
                    driveByEncoder(0.15, 1.95, 0);

                    //Move towards SS
                    driveByEncoder(0.3,-3.9,-3.9);

                    driveByEncoder(0.15, -0.7, -0.7);

                    sleep(500);

                    //Grab skystone
                    robot.pinchyBoi.setPosition(0.525);

                    sleep(750);

                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() - 400);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.33);

                    sleep(100);

                    //Move away from SS
                    driveByEncoder(0.3, 3, 3);

                    //Turn to face foundation
                    driveByEncoder(0.3,-8.1,8.1);
                    driveByEncoder(0.15, 2.5,0);

                    //Drive to foundation
                    driveByEncoder(0.7,-31.5,-31.5);

                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() - 800);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.55);

                    //Turn to face foundation
                    driveByEncoder(0.4,7.75,-7.75);

                    sleep(200);

                    //Drive up to foundation
                    driveByEncoder(0.3,-5.5,-5.5);

                    sleep(200);

                    robot.pinchyBoi.setPosition(0.12);

                    robot.clampyBoi1.setPosition(0.47);
                    robot.clampyBoi2.setPosition(0.52);
                    sleep(100);

                    //Drive to building zone
                    driveByEncoder(0.3,15,15);

                    sleep(200);

                    //Lift clamping arms
                    robot.clampyBoi1.setPosition(0.66);
                    robot.clampyBoi2.setPosition(0.19);

                    //driveByEncoder(0.3,14,0);
                    break;

                case "RIGHT":
                    //Strafe to left
                    driveByEncoder(0.15, 4.85, 0);

                    //Move towards SS
                    driveByEncoder(0.3,-3.9,-3.9);

                    driveByEncoder(0.15, -0.9, -0.9);

                    sleep(500);

                    //Grab skystone
                    robot.pinchyBoi.setPosition(0.525);

                    sleep(750);

                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() - 400);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.33);

                    sleep(100);

                    //Move away from SS
                    driveByEncoder(0.3, 3, 3);

                    //Turn to face foundation
                    driveByEncoder(0.3,-8.3,8.3);
                    driveByEncoder(0.15, 2.5,0);

                    //Drive to foundation
                    driveByEncoder(0.7,-33,-33);

                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() - 800);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.55);

                    //Turn to face foundation
                    driveByEncoder(0.4,7.9,-7.9);

                    sleep(200);

                    //Drive up to foundation
                    driveByEncoder(0.3,-5.5,-5.5);

                    sleep(200);

                    robot.pinchyBoi.setPosition(0.12);

                    robot.clampyBoi1.setPosition(0.47);
                    robot.clampyBoi2.setPosition(0.52);
                    sleep(100);

                    //Drive to building zone
                    driveByEncoder(0.3,16,16);

                    sleep(200);

                    //Lift clamping arms
                    robot.clampyBoi1.setPosition(0.66);
                    robot.clampyBoi2.setPosition(0.19);

                    //driveByEncoder(0.3,14,0);

                    break;

                default:

                    //Strafe to center
                    driveByEncoder(0.15, 1.95, 0);

                    //Move towards SS
                    driveByEncoder(0.3,-3.5,-3.5);

                    driveByEncoder(0.15, -0.7, -0.7);

                    //Grab skystone
                    robot.pinchyBoi.setPosition(0.6);

                    sleep(500);

                    robot.verticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() + 550);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.6);

                    sleep(200);

                    //Move away from SS
                    driveByEncoder(0.3, 3, 3);

                    //Turn to face foundation
                    rotate(85, 0.35);

                    driveByEncoder(0.15, 2.5,0);

                    //Drive to foundation
                    driveByEncoder(0.7,-31.5,-31.5);

                    robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() + 525);
                    robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.verticalMotor.setPower(0.55);

                    //Turn to face foundation
                    rotate(-85, 0.3);

                    //Drive up to foundation
                    driveByEncoder(0.3,-5.5,-5.5);

                    sleep(200);

                    robot.pinchyBoi.setPosition(0.0);

                    robot.clampyBoi1.setPosition(0.47);
                    robot.clampyBoi2.setPosition(0.52);
                    sleep(200);

                    //Drive to building zone
                    driveByEncoder(0.3,15,15);

                    //Lift clamping arms
                    robot.clampyBoi1.setPosition(0.69);
                    robot.clampyBoi2.setPosition(0.16);

                    driveByEncoder(0.3, 28,0);

                    //driveByEncoder(0.3,14,0);
                    break;
            }

            if (closeSkystone) {
                //Adjust as necesssary

            } else {
                //N/A for MEET 1
            }


            if (closeNavWall) {
                //N/A for MEET 1

            } else {
                //Adjust as necessary
            }

            //Drive under Skybridge


            if (dragFoundation) {
                //Drag Foundation

            } else {
                //N/A for MEET 1

            }

            //Place Skystone on Foundation

            //Eventually do this with better stabilization
//            if (closeParkWall) {
//
//            } else {
//
//            }
            //driveByEncoder(0.4,15,0);
//            robot.verticalMotor.setTargetPosition(robot.verticalMotor.getCurrentPosition() + 800);
//            robot.verticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.verticalMotor.setPower(0.55);

            sleep(5000);
            break;


        }

    }



    /*
     **************************************************
     **************USER DEFINED FUNCTIONS:*************
     **************************************************
     */

    private void driveByEncoder(double speed, double leftInches, double rightInches) throws InterruptedException {

        /**********************************************************
         driveByEncoder(0.3, 10.0, 10.0);            // Forward
         driveByEncoder(0.3, -10.0, -10.0);          // Backward
         driveByEncoder(0.3, 0, 10.0);               // Shift Right
         driveByEncoder(0.3, 10.0, 0);               // Shift Left
         driveByEncoder(0.3, 22.0, -22.0);           // Turn Right
         driveByEncoder(0.3, -22.0, 22.0);           // Turn Left
         ***********************************************************/

        String robotAction = "";
        int newLeftTarget;
        int newRightTarget;

        if (leftInches < 0 && rightInches < 0) {
            robotAction = "BACKWARD";
        } else if (leftInches > 0 && rightInches > 0) {
            robotAction = "FORWARD";
        } else if (leftInches > 0 && rightInches == 0) {
            robotAction = "SHIFT_LEFT";
        } else if (leftInches == 0 && rightInches > 0) {
            robotAction = "SHIFT_RIGHT";
        } else if (leftInches < 0 && rightInches > 0) {
            robotAction = "TURN_LEFT";
        } else if (leftInches > 0 && rightInches < 0) {
            robotAction = "TURN_RIGHT";
        } else {
            return;
        }

        // Remember current motors direction, will reset in the end
        DcMotor.Direction dirFL = robot.frontLeftMotor.getDirection();
        DcMotor.Direction dirFR = robot.frontRightMotor.getDirection();
        DcMotor.Direction dirRL = robot.rearLeftMotor.getDirection();
        DcMotor.Direction dirRR = robot.rearRightMotor.getDirection();
        DcMotor.RunMode runModeFL = robot.frontLeftMotor.getMode();
        DcMotor.RunMode runModeFR = robot.frontRightMotor.getMode();
        DcMotor.RunMode runModeRL = robot.rearLeftMotor.getMode();
        DcMotor.RunMode runModeRR = robot.rearRightMotor.getMode();

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // power is removed from the motor, set the current encoder position to zero
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All mortors will move forward
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Determine new target position, and pass to motor controller
        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
        //logMessage("curFL,curFR",  robot.frontLeftMotor.getCurrentPosition() +", "+ robot.frontRightMotor.getCurrentPosition());

        switch (robotAction) {

            case "FORWARD":
                //logMessage("Moving Robot", "FORWARD");
                break;

            case "BACKWARD": // mortor direction aame as FORWAED, because encoder will be "-"
                //logMessage("Moving Robot", "BACKWARD");
                break;

            case "SHIFT_LEFT":
                //logMessage("Moving Robot", "SHIFT_LEFT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //+
                robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //+
                robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //-
                //robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //+
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //+
                //robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //-
                newRightTarget = newLeftTarget;
                break;

            case "SHIFT_RIGHT":
                //logMessage("Moving Robot", "SHIFT_RIGHT");
                robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                robot.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);  //-
                robot.rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);    //-
                robot.rearRightMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);   //+
                //robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);  //-
                //robot.rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);    //-
                //robot.rearRightMotor.setDirection(DcMotor.Direction.REVERSE);   //+
                newLeftTarget = newRightTarget;
                break;

            case "TURN_LEFT":
                //logMessage("Moving Robot", "TURN_LEFT");
                break;

            case "TURN_RIGHT":
                //logMessage("Moving Robot", "TURN_RIGHT");
                break;

        }

        robot.frontLeftMotor.setTargetPosition(newLeftTarget);
        robot.frontRightMotor.setTargetPosition(newRightTarget);
        robot.rearLeftMotor.setTargetPosition(newLeftTarget);
        robot.rearRightMotor.setTargetPosition(newRightTarget);
        //logMessage("newLeftTarget,newRightTarget",  newLeftTarget +", "+ newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the motor speed and start motion
        robot.frontLeftMotor.setPower(Math.abs(speed));
        robot.frontRightMotor.setPower(Math.abs(speed));
        robot.rearLeftMotor.setPower(Math.abs(speed));
        robot.rearRightMotor.setPower(Math.abs(speed));

        //Ramp up motor speed to match target
//        while(power <= speed) {
//            power += RAMP_INCREMENT;
//        }

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy() &&
                robot.rearLeftMotor.isBusy() && robot.rearRightMotor.isBusy()) {

            /*
            logMessage("Path1",  newLeftTarget +", "+ newRightTarget);
            logMessage("Path2",
                    robot.frontLeftMotor.getCurrentPosition() + ", " +
                    robot.frontRightMotor.getCurrentPosition() + ", " +
                            robot.rearLeftMotor.getCurrentPosition() + ", " +
                            robot.rearRightMotor.getCurrentPosition());
            */
        }


        //logMessage("FL,FR,RL,RR",
        //        robot.frontLeftMotor.getCurrentPosition() + ", " +
        //                robot.frontRightMotor.getCurrentPosition() + ", " +
        //                robot.rearLeftMotor.getCurrentPosition() + ", " +
        //                robot.rearRightMotor.getCurrentPosition());

        // Stop all motion;
        stopRobot();

        // Turn off RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset back motors direction
        robot.frontLeftMotor.setDirection(dirFL);
        robot.frontRightMotor.setDirection(dirFR);
        robot.rearLeftMotor.setDirection(dirRL);
        robot.rearRightMotor.setDirection(dirRR);
        robot.frontLeftMotor.setMode(runModeFL);
        robot.frontRightMotor.setMode(runModeFR);
        robot.rearLeftMotor.setMode(runModeRL);
        robot.rearRightMotor.setMode(runModeRR);

    }

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    //Movement path for rightmost two blocks
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

//ORIGINAL CONFIDENCE 0.8
        tfodParameters.minimumConfidence = 0.7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /**
     * Do Skystone detection thingy
     */
    private void stoneDetect(TFObjectDetector tfod) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                if (updatedRecognitions.size() == 2) {

                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            stoneX = (int) recognition.getTop();
                        } else {
                            skystoneX = (int) recognition.getTop();
                        }
                    }

                    //NEW LOGIC FOR DETECTION
                    if (skystoneX == -1) {
                        stonePosition = "RIGHT";

                    } else {
                        if (skystoneX > stoneX)
                            stonePosition = "CENTER";

                        else
                            stonePosition = "LEFT";
                    }

                    telemetry.addData("Skystone Position: ", stonePosition);
//                    telemetry.addData("skystoneX: ", skystoneX);
//                    telemetry.addData("stoneX: ", stoneX);
                    telemetry.update();

                }
            }

        }
    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate
        // left power to left side
        robot.frontLeftMotor.setPower(leftPower);
        robot.rearLeftMotor.setPower(leftPower);

        robot.frontRightMotor.setPower(rightPower);
        robot.rearRightMotor.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }


}