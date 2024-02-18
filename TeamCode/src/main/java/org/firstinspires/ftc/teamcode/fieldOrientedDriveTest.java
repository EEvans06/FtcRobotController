package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="fieldOrientedDriveTest", group="Linear OpMode")
public class fieldOrientedDriveTest extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor rightSlide = null;
    private DcMotor leftSlide = null;

    private DcMotor forebar = null;
//    private Servo leftForebar = null;

    private Servo topClaw = null;
    private Servo botClaw = null;
    private Servo drone = null;

    BNO055IMU imu;
    Orientation angles = new Orientation();

    double initYaw;
    double adjustedYaw;


    public void runOpMode(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit =
                BNO055IMU.AngleUnit.DEGREES;
        parameters.mode =
                BNO055IMU.SensorMode.IMU;
        parameters.accelUnit =
                BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontLeft"); // EH Port: 2
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBackLeft");   // EH Port: 3
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFrontRight");// CH Port: 1
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBackRight");  // CH Port: 3

        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");  // EH Port: 1
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");  // EH Port: 0

        forebar = hardwareMap.get(DcMotor.class, "forebar"); // CH Port: 0
//        leftForebar = hardwareMap.get(Servo.class, "left_forebar");

        topClaw = hardwareMap.get(Servo.class, "topClaw");   // EH Port: 2
        botClaw = hardwareMap.get(Servo.class, "bottomClaw");   // EH Port: 3
        drone = hardwareMap.get(Servo.class, "drone"); //EH Port 4

        forebar.setDirection(DcMotor.Direction.FORWARD);

        forebar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        forebar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        forebar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Starting at", forebar.getCurrentPosition());

        telemetry.update();
        //int maxPosition = (int)(COUNTS_PER_DEGREE *45);

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
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        forebar.setDirection(DcMotor.Direction.FORWARD);
//        forebar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        forebar.setPower(.5);
//        leftForebar.setDirection(Servo.Direction.REVERSE);
//        rightForebar.setPosition(0);
//        leftForebar.setPosition(0);

        topClaw.setDirection(Servo.Direction.REVERSE);
        botClaw.setDirection(Servo.Direction.FORWARD);
        drone.setDirection(Servo.Direction.FORWARD);
        drone.setPosition(.5);
        topClaw.setPosition(.8);
        botClaw.setPosition(.8);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        adjustedYaw = angles.firstAngle-initYaw;
        double zerodYaw = -initYaw+angles.firstAngle;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x) * 180/Math.PI;
        double realTheta;
        realTheta = (360 - zerodYaw) + theta;
        double power = Math.hypot(x, y);

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = (power * cos / maxSinCos + turn);
        double rightFront = (power * sin / maxSinCos - turn);
        double leftBack = (power * sin / maxSinCos + turn);
        double rightBack = (power * cos / maxSinCos - turn);
        if ((power + Math.abs(turn)) > 1) {
            leftFront /= power + turn;
            rightFront /= power - turn;
            leftBack /= power + turn;
            rightBack /= power - turn;
        }

        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightBack);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack); 
    }
}
