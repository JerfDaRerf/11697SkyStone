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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

//===========================================
// ***** Section 1          *****
// ***** Change OpMode Name *****
//===========================================

@TeleOp(name="TestMotors", group="11697")
// @Disabled

public class TestMotors extends LinearOpMode {

//===========================================
// ***** Section 2                *****
// ***** Declare global variables *****
//===========================================

    Hardware robot = new Hardware();

    // Mortors Settings [DO NOT CHANGE]
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder 1440, Andymark = 1120
    static final double     DRIVE_GEAR_REDUCTION    = 1.3 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH          = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    float                      SPEED_RATE           = 0.5f;       // Driving speed rate

    double stabPos = (MAX_POS - MIN_POS) / 2;

    /* Declare Global Variables. */
    private Date today = new Date();
    private DateFormat myDateFormat = new SimpleDateFormat("yy/MM/dd HH:mm:ss");

    /* Insert more stuff as needed
     *  Look at https://tinyurl.com/yb2xco82 for examples */


    /***** Main COde *****/

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        /* Insert initialization code */

        logMessage("Status", "Initialized v4.0a - " +  myDateFormat.format(today));

        waitForStart(); // Wait for the game to start (driver presses PLAY)


        String                     PREVIOUS_MSG        = "";        // MesLog
        float lxValue = 0, lyValue = 0, rxValue = 0, ryValue = 0;

        while (opModeIsActive()) {

            //Control In/Out
            while(gamepad1.a){
                robot.frontFlip.setTargetPosition(robot.frontFlip.getCurrentPosition() - 10);
                robot.frontFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontFlip.setPower(0.5);

                telemetry.addData("Status", "Position Reading: " + robot.frontFlip.getCurrentPosition());
                telemetry.update();
            }

            robot.frontFlip.setPower(0.0);


            while (gamepad1.b) {
                robot.frontFlip.setTargetPosition(robot.frontFlip.getCurrentPosition() + 10);
                robot.frontFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.frontFlip.setPower(0.5);

                telemetry.addData("Status", "Position Reading: " + robot.frontFlip.getCurrentPosition());
                telemetry.update();
            }

            robot.frontFlip.setPower(0.0);

            while(gamepad1.x) {
                robot.verticalMotor.setPower(-0.7);

                telemetry.addData("Status", "Position Reading: " + robot.verticalMotor.getCurrentPosition());
                telemetry.update();
            }

             robot.verticalMotor.setPower(0);

            while(gamepad1.y) {
                   robot.verticalMotor.setPower(0.7);
                   telemetry.addData("Status", "Position Reading: " + robot. verticalMotor.getCurrentPosition());
                telemetry.update();
            }

             robot.verticalMotor.setPower(0);



            if(gamepad1.dpad_up) {
                //robot.leftIn.setPosition(1.0);

                //telemetry.addData("Status: ", "Position Reading: " + robot.leftIn.getPosition());
                telemetry.update();
            }

            if(gamepad1.dpad_down) {
                // robot.rightIn.setPosition(-1.0);

                //telemetry.addData("Status", "Position Reading: " + robot.rightIn.getPosition());
                telemetry.update();
            }

            if(gamepad1.dpad_left) {
                //robot.leftIn.setPosition(0);

                //telemetry.addData("Status", "Position Reading: " + robot.leftIn.getPosition());
                telemetry.update();
            }

            if(gamepad1.dpad_right) {
                //robot.rightIn.setPosition(0);

                //telemetry.addData("Status", "Position Reading: " + robot.rightIn.getPosition());
                telemetry.update();
            }

            //robot.stabbyBoi.setPower(0);

            if(gamepad2.a)  {
                // robot.stabbyBoi.setPower(-0.2);

                //telemetry.addData("Status", "Position Reading: " + robot.stabbyBoi.getCurrentPosition());
                telemetry.update();
            }

            //robot.stabbyBoi.setPower(0);

            if(gamepad2.b)  {
                //robot.stabbyBoi.setPower(0.2);

                //telemetry.addData("Status", "Position Reading: " + robot.stabbyBoi.getCurrentPosition());
                telemetry.update();
            }

/*
            if(gamepad2.a) {
                robot.stabbyBoi.setPosition(0);
            } else if (gamepad2.b) {
                robot.stabbyBoi.setPosition(-5.0);
            }

*/
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop

            //Long MAX READING: ~4800
            //Long MIN READING: ~-100

            //Vert MAX READING: 1290
            //Vert MIN READING:0

            //Spinny MAX READING:
            //Spinny MIN READING: 105

        }

        logMessage("Done", "Stopped!");
    }



//===========================================
// ***** Section 5              *****
// ***** User Defined Functions *****
//===========================================

    private void stopRobot() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    private void logMessage (String myDescription, String myMessage) throws InterruptedException {

        telemetry.addData(myDescription, myMessage);
        telemetry.update();
        RobotLog.d("11697CW - " + myDescription + " : " + myMessage);


    }

    private float getJoystickValue (float rawValue) {
        //clip the power values so that it only goes from -1 to 1
        rawValue = Range.clip(rawValue, -1, 1);

        if (Math.abs(rawValue) < 0.1) {
            return 0;
        } else {
            return rawValue * SPEED_RATE;
        }
    }

//===========================================
// ***** Section 6              *****
// ***** Backup Un-used Code    *****
//===========================================

/********************************************

 if (gamepad1.dpad_up) {
 V_POSITION += INCREMENT ;
 if (V_POSITION >= robot.VArmHigh ) {
 V_POSITION = robot.VArmHigh;
 }
 robot.vArm.setPosition(V_POSITION);
 telemetry.addData("UP >> Servo Position", "%5.2f", V_POSITION);
 telemetry.update();
 //sleep(CYCLE_MS);
 robot.waitForTick(CYCLE_MS);
 }
 if (gamepad1.dpad_down) {
 V_POSITION -= INCREMENT ;
 if (V_POSITION <= robot.VArmLow ) {
 V_POSITION = robot.VArmLow;
 }
 robot.vArm.setPosition(V_POSITION);
 telemetry.addData("DOWN >> Servo Position", "%5.2f", V_POSITION);
 telemetry.update();
 //sleep(CYCLE_MS);
 robot.waitForTick(CYCLE_MS);
 }

 ********************************************/





}

