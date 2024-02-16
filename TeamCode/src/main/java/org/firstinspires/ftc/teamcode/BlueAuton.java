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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;



/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Blue Side Autonomous", group="Robot")
//@Disabled
public class BlueAuton extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor rightSlide = null;
    private DcMotor leftSlide = null;

    private DcMotor forebar = null;
//    private Servo leftForebar = null;

    private Servo topClaw = null;
    private Servo botClaw = null;

    private ElapsedTime runtime = new ElapsedTime();

    private int order = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20;     // 20:1 gear ratio.
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


//    /********** Copied Code **********/
//    double cX = 0;
//    double cY = 0;
//    double width = 0;
//    double distance = getDistance(width);

//    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
//    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 720 ; // height of wanted camera resolution
//
//    // Calculate the distance using the formula
//    public static final double objectWidthInRealWorldUnits = 3.25                ;  // Replace with the actual width of the object in real-world units
//    public static final double focalLength = 1473.23;  // Replace with the focal length of the camera in pixels
//    /********************/
//1444.5


    @Override
    public void runOpMode() {

        /*********   Copied Code ******/
//        initOpenCV();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        /*******   Copied Code ******/


        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        rightSlide = hardwareMap.get(DcMotor.class, "right_slide");  // EH Port: 1
        leftSlide = hardwareMap.get(DcMotor.class, "left_slide");  // EH Port: 0

        forebar = hardwareMap.get(DcMotor.class, "forebar"); // CH Port: 0
//        leftForebar = hardwareMap.get(Servo.class, "left_forebar");

        topClaw = hardwareMap.get(Servo.class, "top_claw");   // EH Port: 2
        botClaw = hardwareMap.get(Servo.class, "bottom_claw");   // EH Port: 3
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        forebar.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forebar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        telemetry.update();

        topClaw.setDirection(Servo.Direction.REVERSE);
        botClaw.setDirection(Servo.Direction.FORWARD);
        topClaw.setPosition(.8);
        botClaw.setPosition(.8);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        while (opModeIsActive()) {
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (distance));
//            telemetry.update();
//        }

//        if(distance > 10)
//
//        {
//            encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);
////
////            encoderDrive(DRIVE_SPEED, 33, 33, 33, 33, 10.0);  // S1: Forward 30 Inches with 5 Sec timeout
////            encoderDrive(DRIVE_SPEED, -25, -25, -25, -25, 10.0);  // S1: Reverse 30 Inches with 5 Sec timeout
////            encoderDrive(TURN_SPEED, 25, -25, 25, -25, 8.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
////            encoderDrive(DRIVE_SPEED, 35, 35, 35, 35, 8.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//        }
//        leftSlide.setPower(1);
//        rightSlide.setPower(1);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  23,  23, 23, 23,
                0, 0, 0, 0, 0, 5.0);  // S1: Forward 30 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   -17, 17, -17, 17,
                0, 0, 0, 0, 0, 8.0); //        encoderDrive(DRIVE_SPEED,  -20,  -20, -20, -20, 10.0);  // S1: Reverse 30 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED,  32,  32, 32, 32,
                0, 0, 0, 0, 0,   5.0);  // S1: Forward 30 Inches with 5 Sec timeout
        order = 1;
        while (order == 1 && opModeIsActive()) {
            encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                    925, 943, 696, 0.3, 0.2,5.0);  // S1: Forward 30 Inches with 5 Sec timeout
            order = 0;
            break;
        }
        topClaw.setPosition(1);
        sleep(1000);
        botClaw.setPosition(1);
        sleep(1000);
        botClaw.setPosition(.8);
        topClaw.setPosition(.8);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forebar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        order = 2;
        while (order == 2 && opModeIsActive()) {
            encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                    -917, -934, - 670, 0.3, 0.2,5.0);
            order = 0;
            break;
        }
        encoderDrive(DRIVE_SPEED, -5, -5, -5, -5,
                0, 0, 0, 0, 0,5.0);



//      1822, 1871, 863
//        encoderDrive(DRIVE_SPEED,  0,  0, 0, 0, 600, 600, 0,  5.0);  // S1: Forward 30 Inches with 5 Sec timeout


// encoderDrive(TURN_SPEED,   23, -23, 23, -23, 8.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, 35, -35, -35, 35, 8.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             int leftEncoder, int rightEncoder,
                             int forebarEncoder,
                             double slideSpeed,
                             double forebarSpeed,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
//            newLeftSlideTarget = leftSlide.getCurrentPosition() + (int) (leftSlideInches * COUNTS_PER_INCH);
//            newrightSlideTarget = rightSlide.getCurrentPosition() + (int) (rightSlideInches * COUNTS_PER_INCH);
//            newForebarTarget = forebar.getCurrentPosition() + (int) (forebarInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);
            leftSlide.setTargetPosition(leftEncoder);
            rightSlide.setTargetPosition(rightEncoder);
            forebar.setTargetPosition(forebarEncoder);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            forebar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftSlide.setPower(Math.abs(slideSpeed));
            rightSlide.setPower(Math.abs(slideSpeed));
            forebar.setPower(Math.abs(forebarSpeed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                            leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d",
                        newLeftFrontTarget, newRightFrontTarget,
                        newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(),
                        leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            while(opModeIsActive() &&
                    (runtime.seconds()<timeoutS)&&
                    (leftSlide.isBusy() && rightSlide.isBusy())){
                telemetry.addData("Running to slide position", "%7d : %7d",
                        leftEncoder, rightEncoder);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition());
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftSlide.setPower(0);
            rightSlide.setPower(0);
            forebar.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


}

