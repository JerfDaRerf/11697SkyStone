// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="TestGyro", group="11697")
//@Disabled
public class TestGyro extends LinearOpMode
{
    Hardware robot = new Hardware();
    DcMotor                 leftMotor, rightMotor;
    TouchSensor             touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .3, correction;
    //boolean                 aButton, bButton, touched;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
//        leftMotor = hardwareMap.dcMotor.get("left_motor");
//        rightMotor = hardwareMap.dcMotor.get("right_motor");
//
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("(God)Mode: ", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.

        while(opModeIsActive()) {
            // Use gyro to drive in a straight line.
            correction = checkDirection();

            telemetry.addData("1 imu heading to failure", lastAngles.firstAngle);
            telemetry.addData("2 global heading (heating*)", globalAngle);
            telemetry.addData("3 correction(al officer)", correction);
            telemetry.update();

            robot.frontLeftMotor.setPower(-(    power + correction));
            robot.rearLeftMotor.setPower(power - correction);

            robot.frontRightMotor.setPower(power - correction);
            robot.rearRightMotor.setPower(-(power + correction));

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            if (gamepad1.x|| gamepad1.a || gamepad1.b) {
                // backup
                robot.frontLeftMotor.setPower(power);
                robot.frontRightMotor.setPower(power);
                robot.rearLeftMotor.setPower(power);
                robot.rearRightMotor.setPower(power);

                sleep(500);

                // stop
                robot.frontLeftMotor.setPower(0);
                robot.frontRightMotor.setPower(0);
                robot.rearLeftMotor.setPower(0);
                robot.rearRightMotor.setPower(0);

                // turn 90 degrees right.
                if (gamepad1.x || gamepad1.a) rotate(-85, power);

                // turn 90 degrees left.
                if (gamepad1.b) rotate(85, power);
            }
        }

        // turn the motors off.
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.rearLeftMotor.setPower(0);
        robot.rearRightMotor.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */

    //ORIGINAL PRIORITY: ZYX
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES);

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

        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES);

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
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}