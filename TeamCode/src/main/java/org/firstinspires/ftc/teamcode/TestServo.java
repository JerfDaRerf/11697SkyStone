/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "TestServo", group = "11697")
//@Disabled
public class TestServo extends LinearOpMode {

    Hardware robot = new Hardware();

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo  servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double curPosition1 = 0.0;
    double curPosition2 = 0.0;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){
            curPosition1 = robot.clampyBoi1.getPosition();
            curPosition2 = robot.clampyBoi2.getPosition();
/*
            robot.vertMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.vertMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           // robot.vertMotor.setTargetPosition(-2000);
            robot.vertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.vertMotor.setPower(1);
            while (robot.vertMotor.isBusy() ) {
                telemetry.addData("Path1",  "Running to " + robot.vertMotor.getCurrentPosition());
                telemetry.update();
            }

*/
/*            // slew the servo, according to the rampUp (direction) variable.
            if (gamepad1.a) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    //rampUp = !rampUp;   // Switch ramp direction
                }
            }
            if (gamepad1.b) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                    //rampUp = !rampUp;  // Switch ramp direction
                }
            }
*/
/*
            while(gamepad1.x) {
                    robot.vertMotor.setPower(0.5);

                    telemetry.addData("Status", "Position Reading: " + robot.vertMotor.getCurrentPosition());
                    telemetry.update();
            }

            while(gamepad1.y) {
                robot.vertMotor.setPower(-0.5);

                telemetry.addData("Status", "Position Reading: " + robot.vertMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.vertMotor.setPower(0);
*/

            /*while(gamepad1.left_bumper) {
                curPosition += 0.01;
                servo.setPosition(curPosition);

                telemetry.addData("Status", "Position Reading: " + servo.getPosition());
                telemetry.update();
            }

            while(gamepad1.right_bumper) {
                curPosition -= 0.01;
                servo.setPosition(curPosition);

                telemetry.addData("Status", "Position Reading: " + servo.getPosition());
                telemetry.update();
            } */

//            while(gamepad1.a) {
//                curPosition1 += 0.01;
//
//                robot.dumpyBoi.setPosition(curPosition1);
//                telemetry.addData("Status: ", "Position Reading(dumpyBoi): " + robot.dumpyBoi.getPosition());
//                telemetry.update();
//
//            }
//            while(gamepad1.b) {
//                curPosition1 -= 0.01;
//
//                robot.dumpyBoi.setPosition(curPosition1);
//                telemetry.addData("Status: ", "Position Reading(dumpyBoi): " + robot.dumpyBoi.getPosition());
//                telemetry.update();
//
//            }


            while(gamepad1.x) {
                curPosition2 += 0.01;

                robot.pinchyBoi.setPosition(robot.pinchyBoi.getPosition() + 0.01);
                telemetry.addData("Status: ", "Position Reading(clampyBoi2): " + robot.pinchyBoi.getPosition());
                telemetry.update();

            }
            while(gamepad1.y) {
                curPosition2 -= 0.01;

                robot.pinchyBoi.setPosition(robot.pinchyBoi.getPosition() - 0.01);
                telemetry.addData("Status: ", "Position Reading(clampyBoi2): " + robot.pinchyBoi.getPosition());
                telemetry.update();

            }
            while(gamepad1.a) {
                curPosition2 += 0.01;

                robot.rightSide.setPosition(robot.rightSide.getPosition() + 0.01);
                telemetry.addData("Status: ", "Position Reading(rightSide): " + robot.rightSide.getPosition());
                telemetry.update();

            }
            while(gamepad1.b) {
                curPosition2 -= 0.01;

                robot.rightSide.setPosition(robot.rightSide.getPosition() - 0.01);
                telemetry.addData("Status: ", "Position Reading(rightSide): " + robot.rightSide.getPosition());
                telemetry.update();
            }
            while(gamepad1.left_bumper) {
                curPosition2 += 0.01;

                robot.leftSide.setPosition(robot.leftSide.getPosition() + 0.01);
                telemetry.addData("Status: ", "Position Reading(leftSide): " + robot.leftSide.getPosition());
                telemetry.update();

            }
            while(gamepad1.right_bumper) {
                curPosition2 -= 0.01;

                robot.leftSide.setPosition(robot.leftSide.getPosition() - 0.01);
                telemetry.addData("Status: ", "Position Reading(leftSide6): " + robot.leftSide.getPosition());
                telemetry.update();
            }


            // Display the current value
            // telemetry.addData("Servo Position", "%5.2f", position);
            //telemetry.addData(">", "Press Stop to end test." );
            //telemetry.update();

            // Set the servo to the new position and pause;
            //servo.setPosition(position);
            //sleep(CYCLE_MS);
            idle();
        }


    }
}