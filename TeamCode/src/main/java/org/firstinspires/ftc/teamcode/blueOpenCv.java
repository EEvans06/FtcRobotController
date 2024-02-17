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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// New imports for openCV
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

import java.util.List;

import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="blueOpenCv", group="Robot")
//@Disabled
public class blueOpenCv extends LinearOpMode {

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightBackDrive  = null;

    private DcMotor rightSlide = null;
    private DcMotor leftSlide = null;

    private DcMotor forebar = null;
//    private Servo leftForebar = null;

    private Servo topClaw = null;
    private Servo botClaw = null;

    private ElapsedTime runtime = new ElapsedTime();

    private WebcamName webcam1 = null;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 20  ;     // 20:1 gear ratio.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private double  order = 0;


    // New variables for OpenCV
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 960; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.375;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1431.11;  // Replace with the focal length of the camera in pixels

    double spikeTarget = 3;
    public double maxArea = 0;

    @Override
    public void runOpMode() {

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        initOpenCV();
        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");  // EH Port: 1
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");  // EH Port: 0

        forebar = hardwareMap.get(DcMotor.class, "forebar"); // CH Port: 0
//        leftForebar = hardwareMap.get(Servo.class, "left_forebar");

        topClaw = hardwareMap.get(Servo.class, "topClaw");   // EH Port: 2
        botClaw = hardwareMap.get(Servo.class, "bottomClaw");   // EH Port: 3
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
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
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("cX is equal to", cX);
        telemetry.addData("maxArea is equal to", maxArea);
        telemetry.update();
        telemetry.update();

        topClaw.setDirection(Servo.Direction.REVERSE);
        botClaw.setDirection(Servo.Direction.FORWARD);
        topClaw.setPosition(.8);
        botClaw.setPosition(.8);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.addData("cX is equal to", cX);
            telemetry.addData("maxArea is equal to", maxArea);
        //            telemetry.update();
            double distance = getDistance(width);

            telemetry.addData("Spike Target", spikeTarget);

            telemetry.update();


            controlHubCam.stopStreaming();

            telemetry.addData("Spike Target", spikeTarget);

            telemetry.update();




            if (spikeTarget == 1&&order==0) {
                encoderDrive(DRIVE_SPEED, -20, 20, 20, -20,
                        0, 0, 0, 0, 0, 5.0);//strafe left
                encoderDrive(DRIVE_SPEED, 23, 23, 23, 23,
                        0, 0, 0, 0, 0, 5.0);//forward
                encoderDrive(TURN_SPEED, -16, 16, -16, 16,
                        0, 0, 0, 0, 0, 5.0);//turn right
                encoderDrive(DRIVE_SPEED, 3, 3, 3, 3,
                        0, 0, 0, 0, 0, 5.0);//forward
                topClaw.setPosition(1);
                order = 1;
                while (order == 1 && opModeIsActive()) {
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            0, 0, 210, 0, 0.3, 5.0);
                    encoderDrive(DRIVE_SPEED, -10, -10, -10, -10,
                            0, 0, 0, 0, 0, 5.0);

                    sleep(1000);
                    topClaw.setPosition(.8);
                    order = 2;
                }
                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                forebar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                encoderDrive(DRIVE_SPEED, 13, -13, -13, 13,
                        0, 0, 0, 0, 0, 5.0); //strafe left

                while (order == 2 && opModeIsActive()) {
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            830, 910, 850, 0.3, 0.2, 5.0);
                    order = 3;
                }
                topClaw.setPosition(1);
                botClaw.setPosition(1);

                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                forebar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                topClaw.setPosition(0.8);
                botClaw.setPosition(0.8);

                while (order == 3 && opModeIsActive()) {
                    topClaw.setPosition(1);
                    botClaw.setPosition(1);
                    sleep(1000);
                    topClaw.setPosition(.8);
                    botClaw.setPosition(.8);
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            -917, -934, -670, 0.3, .2, 5.0);
                    order = 4;
                }
            } else if (spikeTarget == 2 && order==0) {
                encoderDrive(DRIVE_SPEED, 22, 22, 22, 22,
                        0, 0, 0, 0, 0, 5.0);//Forward
                topClaw.setPosition(1);
                order = 1;
                while (order == 1 && opModeIsActive()) {
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            0, 0, 140, 0, .2, 5.0);//raise forebar
                    encoderDrive(DRIVE_SPEED, -5, -5, -5, -5,
                            0, 0, 140, 0, .2, 5.0);//drive backwards
                    encoderDrive(TURN_SPEED, -17, 17, -17, 17,
                            0, 0, 0, 0, 0, 5.0);//turn right
                    order = 4;
                }
                topClaw.setPosition(.8);
                encoderDrive(DRIVE_SPEED, -18, -18, -18, -18,
                        0, 0, 0, 0, 0, 5.0);//Forward
                while (order == 4 && opModeIsActive()){
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            830, 910, 850, 0.3, 0.2, 5.0);//Raise slides and forebar to the board
                    order = 5;
                }
                topClaw.setPosition(1);
                botClaw.setPosition(1);
                sleep(1000);
                topClaw.setPosition(.8);
                botClaw.setPosition(.8);

                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                forebar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                while(order == 5){
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            -917, -934, -670, 0.3, .2, 5.0);//lower slides and forbar
                }

            }
            else if (spikeTarget == 3 && order==0){
                encoderDrive(DRIVE_SPEED, 23, 23, 23, 23,
                        0, 0, 0, 0, 0, 5.0);//Drive forward
                encoderDrive(TURN_SPEED, 16, -16, 16, -16,
                        0, 0, 0, 0, 0, 5.0);//Turn 90 degrees to the right
                encoderDrive(DRIVE_SPEED, 5, 5, 5, 5,
                        0, 0, 0, 0, 0, 5.0);//Drive forward
                topClaw.setPosition(1);//Opening bottom claw to release PP
                order = 1;

                while (order == 1 && opModeIsActive()){
                    encoderDrive(DRIVE_SPEED, -23, -23, -23, -23,
                            0, 0, 210, 0, .3, 5.0);//Raise the forebar to release the PP
                    topClaw.setPosition(.8);
                    //as well as move backwards to board
                    sleep(500);
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            830, 910, 850, 0.3, 0.2, 5.0);//Raise slides and forebar to the board
                    order = 2;
                }
                topClaw.setPosition(1);
                botClaw.setPosition(1);

                rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                forebar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                while (order == 2 && opModeIsActive()){
                    topClaw.setPosition(.8);
                    botClaw.setPosition(.8);
                    encoderDrive(DRIVE_SPEED, 0, 0, 0, 0,
                            -917, -934, -670, 0.3, .2, 5.0);//lower slides and forbar
                }
            }
        }
        controlHubCam.stopStreaming();
    }





        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  18,  18, 18, 18, 5.0);  // S1: Forward 30 Inches with 5 Sec timeout
//        encoderDrive(DRIVE_SPEED,  -21,  -21, -21, -21, 5.0);  // S1: Reverse 30 Inches with 5 Sec timeout
//        encoderDrive(DRIVE_SPEED,  2,  2, 2, 2, 5.0);  // S1: Reverse 30 Inches with 5 Sec timeout
//        sleep(5000);
//        encoderDrive(DRIVE_SPEED,  85,  -85,  -85, 85, 5.0);  // S1: Reverse 30 Inches with 5 Sec timeout
//        encoderDrive(DRIVE_SPEED,  -5,  -5, -5, -5, 5.0);  // S1: Reverse 30 Inches with 5 Sec timeout        //encoderDrive(DRIVE_SPEED, 50, 50, 50, 50, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

//        encoderDrive(TURN_SPEED,   -7, 7, -7, 7, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // pause to display final telemetry message.


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
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

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
                telemetry.addData("Running to",  " %7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget,
                        newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
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
    private void initOpenCV() {

        // Create an instance of the camera
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(webcam1);

        controlHubCam.setPipeline(new blueOpenCv.YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
class YellowBlobDetectionPipeline extends OpenCvPipeline {
    Mat hsvFrame = new Mat();
    //        Mat YCrCb = new Mat();
    Mat hierarchy = new Mat();
    Mat yellowMask = new Mat();
//        Mat Cr = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Preprocess the frame to detect yellow regions
        Mat yellowMask = preprocessFrame(input);

        // Find contours of the detected yellow regions
        List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest yellow contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
            // Calculate the width of the bounding box
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String areaLabel = "Area: " + (int) maxArea + "pixels";
            Imgproc.putText(input, areaLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 2);
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
        }
        if ((int) cX > 180 && (int) cX < 800 && (int)maxArea>12000) {
            spikeTarget = 2;
        } else if ((int) cX < 300 && (int)maxArea>12000) {
            spikeTarget = 1;
        } else {
            spikeTarget = 3;
            telemetry.addData("Else Spike target = 3","Hello world");
            telemetry.update();
        }

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
//            Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

//            Imgproc.cvtColor(frame, YCrCb, Imgproc.COLOR_BGR);
//        Scalar lowerYellow = new Scalar(100, 100, 100);
//        Scalar upperYellow = new Scalar(180, 255, 255);
        Scalar lowerYellow = new Scalar(0, 100, 100);
        Scalar upperYellow = new Scalar(120, 255, 255);

//            Scalar lowerRed = new Scalar(0, 0, 0);
//            Scalar upperRed = new Scalar(255, 255, 255);

//            Core.extractChannel(YCrCb, Cr, 2);

//            Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        MatOfPoint largestContour = null;
        maxArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

}



}



