/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Op Mode: 1 Player", group="Linear Opmode")

public class OmniOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx slide = null;
    private Servo rightClaw = null;
    private Servo leftClaw = null;
    boolean slow = true;


    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_MM       = 112.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = WHEEL_DIAMETER_MM/25.4 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.45;
    static final int     maxSlideHeight          = 4000;
    static final int     midSlideHeight          = 2800;
    static final int     lowSlideHeight          = 1650;
    static final int     minSlideHeight          = 10;

    static final double     SLIDE_SPEED             = 0.65;


    public void extendSlide(double maxDriveSpeed, int slideHeight){
        int slideTarget = slideHeight;
        slide.setTargetPosition(slideTarget);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        maxDriveSpeed = Math.abs(maxDriveSpeed);
        while (opModeIsActive() && slide.isBusy() && !(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)){
            slide.setPower(maxDriveSpeed);
        }

        slide.setPower(0.0);
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

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        //direction keywords are REVERSE and FORWARD

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        extendSlide(SLIDE_SPEED, minSlideHeight);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;



//            boolean clawRight = gamepad1.right_bumper;
//            boolean clawLeft = gamepad1.left_bumper;

            if(gamepad1.right_bumper && gamepad1.left_bumper){
                rightClaw.setPosition(0.5);
                leftClaw.setPosition(0.5);
            }
            else if(gamepad1.right_bumper){
                rightClaw.setPosition(0.25);
                leftClaw.setPosition(0.75);
            }
            else if(gamepad1.left_bumper){
                rightClaw.setPosition(1.0);
                leftClaw.setPosition(0.0);
            }


//            while(gamepad1.right_trigger > 0 && slide.getCurrentPosition() < maxSlideHeight){
//
//                if(slide.getCurrentPosition() > maxSlideHeight - 10){
//                    slide.setVelocity(1 * 28);
//                } else {
//                    slide.setVelocity(30.0 * 28);
//                }
//            }
//            while(gamepad1.left_trigger > 0 && slide.getCurrentPosition() > 50){
//                slide.setVelocity(-15.0 * 28);
//            }
//            if(gamepad1.x || slide.getCurrentPosition() > 0 || slide.getCurrentPosition() < -2057){
//                slide.setVelocity(0);
//            }
            boolean resetSlide = false;
            while(gamepad1.back){
                slide.setPower(-SLIDE_SPEED);
                resetSlide = true;
            }
            if(resetSlide){
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                resetSlide = false;
            }


            if(gamepad1.right_trigger > 0 && slide.getCurrentPosition() < maxSlideHeight){
                int slideTarget = maxSlideHeight;
                slide.setTargetPosition(slideTarget);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(SLIDE_SPEED);
            }
            else if(gamepad1.left_trigger > 0 && slide.getCurrentPosition() > 50){
                int slideTarget = minSlideHeight;
                slide.setTargetPosition(slideTarget);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(-SLIDE_SPEED);
            }
            else if((gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left || slide.getCurrentPosition() <= 50 || slide.getCurrentPosition() >= maxSlideHeight){
                slide.setPower(0.0);
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad1.y){
                //top
                extendSlide(SLIDE_SPEED, maxSlideHeight);
            }
            else if(gamepad1.x){
                //middle
                extendSlide(SLIDE_SPEED, midSlideHeight);
            }
            else if(gamepad1.b){
                //short
                extendSlide(SLIDE_SPEED, lowSlideHeight);
            }
            else if(gamepad1.a){
                retractSlide(SLIDE_SPEED);
            }


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels

            if(gamepad1.start){
                slow = !slow;
            }
            if(slow){
                leftFrontDrive.setPower(leftFrontPower / 2);
                rightFrontDrive.setPower(rightFrontPower/2);
                leftBackDrive.setPower(leftBackPower/2);
                rightBackDrive.setPower(rightBackPower/2);
            }
            else if(!slow){
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "is slow: " + Boolean.toString(slow));
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("current  position", "%4d, %4d", slide.getCurrentPosition(), slide.getCurrentPosition());
            telemetry.update();
        }


    }



}