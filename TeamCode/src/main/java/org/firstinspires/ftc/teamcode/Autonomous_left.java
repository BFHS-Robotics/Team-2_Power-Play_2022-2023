/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Autonomous(name="Autonomous left", group="Robot")
public class Autonomous_left extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx slide = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftFrontTarget    = 0;
    private int     rightFrontTarget   = 0;
    private int     leftBackTarget    = 0;
    private int     rightBackTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     SLIDE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    static final int     maxSlideHeight          = 4000;
    static final int     midSlideHeight          = 2800;
    static final int     lowSlideHeight          = 1750;
    static final int     minSlideHeight          = 5;
    static final int     stack5Height            = 700;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "br");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (Display Gyro value while waiting)
        boolean run = true;
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
            if(run){
                run = false;
                closeClaw(runtime.milliseconds());
            }
        }//i love fortnite

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review
        waitForStart();
        runtime.reset();

        driveStraight(DRIVE_SPEED, 2.0, 0);
        strafeRight(DRIVE_SPEED, 10.0);
        extendSlide(SLIDE_SPEED, lowSlideHeight);
        driveStraight(DRIVE_SPEED, 5.5, 0);
        openClaw(runtime.milliseconds());
        //scored first one
        driveStraight(DRIVE_SPEED, -4.0, 0.0);
        strafeLeft(DRIVE_SPEED, 11.5);
        retractSlide(SLIDE_SPEED);
        driveStraight(DRIVE_SPEED - 0.15, 58.5, 0.0);
        //plow beacon out of the way
        driveStraight(DRIVE_SPEED, -8.75, 0.0);
        turnToHeading(TURN_SPEED, 90.0);
        extendSlide(SLIDE_SPEED, stack5Height);
        driveStraight(DRIVE_SPEED, 24.25, 90.0);
        extendSlide(SLIDE_SPEED, stack5Height);
        closeClaw(runtime.milliseconds());
        //pick up other cone from stack
        extendSlide(SLIDE_SPEED, lowSlideHeight);
        driveStraight(DRIVE_SPEED, -21.5, 90.0);
        extendSlide(SLIDE_SPEED, lowSlideHeight + 70);
        strafeLeft(DRIVE_SPEED, 11.25);
        wait(runtime.milliseconds(), 500);
        openClaw(runtime.milliseconds());
        //score x2
        driveStraight(DRIVE_SPEED, -3.0, 90);
        retractSlide(SLIDE_SPEED);
        //fix parking
//        driveStraight(DRIVE_SPEED, 2.0, 0);
//        strafeRight(DRIVE_SPEED, 10.0);
//        extendSlide(SLIDE_SPEED, lowSlideHeight);
//        driveStraight(DRIVE_SPEED, 7.0, 0);
//        openClaw(runtime.milliseconds());
//        //scored first one
//        driveStraight(DRIVE_SPEED, -4.0, 0.0);
//        strafeLeft(DRIVE_SPEED, 12.5);
//        retractSlide(SLIDE_SPEED);
//        driveStraight(DRIVE_SPEED - 0.15, 57.0, 0.0);
//        //plow beacon out of the way
//        driveStraight(DRIVE_SPEED, -9.25, 0.0);
//        turnToHeading(TURN_SPEED, 90.0);
//        extendSlide(SLIDE_SPEED, stack5Height);
//        driveStraight(DRIVE_SPEED, 23.75, 90.0);
//        extendSlide(SLIDE_SPEED, stack5Height);
//        closeClaw(runtime.milliseconds());
//        //pick up other cone from stack
//        extendSlide(SLIDE_SPEED, lowSlideHeight);
//        driveStraight(DRIVE_SPEED, -24, 90.0);
//        extendSlide(SLIDE_SPEED, lowSlideHeight + 200);
//        strafeLeft(DRIVE_SPEED, 12.0);
//        openClaw(runtime.milliseconds());
//        //score x2
//        driveStraight(DRIVE_SPEED, -3.0, 90);



        telemetry.addData("slide  position", "%4d, %4d", slide.getCurrentPosition(), slide.getCurrentPosition());
        telemetry.update();
        sleep(1000);  // Pause to display last telemetry message.
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void closeClaw(double milliseconds){
        rightClaw.setPosition(0.25);
        leftClaw.setPosition(0.75);
        wait(milliseconds, 1250);
    }
    public void openClaw(double milliseconds){
        rightClaw.setPosition(1.0);
        leftClaw.setPosition(0.0);
        wait(milliseconds, 1250);
    }
    public void centerClaw(double milliseconds){
        rightClaw.setPosition(0.5);
        leftClaw.setPosition(0.5);
        wait(milliseconds, 1000);
    }
    public void wait(double milliseconds, double wait){
        while(runtime.milliseconds() < milliseconds + wait && opModeIsActive()){
        }
    }
    public void stopMotors(){
        leftFrontDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
    }
    public void extendSlide(double maxDriveSpeed, int slideHeight){
        int slideTarget = slideHeight;
        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxDriveSpeed = Math.abs(maxDriveSpeed);
        while (opModeIsActive() && slide.isBusy() && !(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
            slide.setPower(maxDriveSpeed);
//            slide.setVelocity(60*28.0);
        }


        slide.setPower(0.0);
//        slide.setVelocity(0.0);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void retractSlide(double maxDriveSpeed){
        int slideTarget = minSlideHeight;
        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxDriveSpeed = Math.abs(maxDriveSpeed);
        while (opModeIsActive() && slide.isBusy()&& !(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
            slide.setPower(-maxDriveSpeed);
        }

        slide.setPower(0.0);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRight(double power, double distance){
        int moveCounts = (int)(distance * COUNTS_PER_INCH);
        int moveTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
        power = Math.abs(power);

        while(leftFrontDrive.getCurrentPosition() < moveTarget && opModeIsActive()){
            //fr back, fl forward, br forward, bl back
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
        stopMotors();

    }
    public void strafeLeft(double power, double distance){
        int moveCounts = (int)(distance * COUNTS_PER_INCH);
        int moveTarget = leftFrontDrive.getCurrentPosition() - moveCounts;
        power = Math.abs(power);

        while(leftFrontDrive.getCurrentPosition() > moveTarget && opModeIsActive()){
            //fr forward, fl back, br back, bl forward
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
        stopMotors();


    }
    /**
     *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            leftBackDrive.setTargetPosition(leftBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and all motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontDrive.setPower(leftSpeed);
        leftBackDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        rightBackDrive.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}