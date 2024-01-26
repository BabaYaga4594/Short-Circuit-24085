package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RedTopRight")
public class AutoRed extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightVP;
    private DcMotor leftVP;
    private Servo claw;
    private Servo claw2;
    private Servo armServo;
    private Servo armServo2;
    private int leftFrontPos;
    private int leftBackPos;
    private int rightFrontPos;
    private int rightBackPos;
    private int leftVPPos;
    private int rightVPPos;
    OpenCvWebcam webcam;

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    PropDetection propDetection = new PropDetection(telemetry, false);

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        armServo = hardwareMap.get(Servo.class, "armServo");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;
        leftVPPos = 0;
        rightVPPos = 0;

        leftVP = hardwareMap.get(DcMotor.class, "viperLeft");
        rightVP = hardwareMap.get(DcMotor.class, "viperRight");
        leftVP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftVP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVP.setDirection(DcMotor.Direction.REVERSE);
        rightVP.setDirection(DcMotor.Direction.FORWARD);



        // Initialize the drive system variables.
        robot.init(hardwareMap, telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(propDetection);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            PropPosition position = propDetection.getPosition();
            if (position == PropPosition.LEFT) {
                telemetry.addData("Prop Position", "LEFT");
                telemetry.update();
/*
                // Reset arm servos and claw servos
                armServo.setPosition();
                claw.setPosition();
                claw2.setPosition();
                // Move forward a little
                regDrive(100,100,0.5);
                // Turn left 90 degrees
                robot.leftTurn(0.6);
                // Move forward
                regDrive(100,100,0.5);
                // Move backward
                regDrive(100,100,0.5);
                // Open servo to place pixel on spike mark
                claw.setPosition();
                // Move backward
                regDrive(100,100,0.5);
                // Wait 1 second
                sleep(1000);
                // Close servo
                claw.setPosition();
                // Turn 180 degrees
                robot.rightTurn(0.6);
                robot.rightTurn(0.6);
                // Move forward
                regDrive(100,100,0.5);
                // Extend viperslides slightly and slowly
                VPmove(100,100,0.5);
                // Move armServo and armServo2 to a 60-degree angle
                armServo.setPosition(60.0 / 180.0);
                armServo2.setPosition(60.0 / 180.0);
                // Wait 1 second
                sleep(1000);
                // Open claw2 servo
                claw2.setPosition(0.7);
                // Move back a little
                regDrive(100,100,1);
                // Close claw2 and move arm back to normal
                claw2.setPosition(0.2);
                armServo.setPosition(0.5);
                armServo2.setPosition(0.5);
                // Retract viperslides
                VPmove(0,0,0.5);
                // Strafe to the right a bit
                specDrive(0,0,0,0,0.5);
                // Move forward a little
                regDrive(100,100,0.5);
                break;
*/
            } else if (position == PropPosition.MIDDLE) {
                telemetry.addData("Prop Position", "MIDDLE");
                telemetry.update();
/*
                // Reset arm servos and claw servos
                armServo.setPosition();
                claw.setPosition();
                claw2.setPosition();
                // Move forward
                regDrive(100,100,1);
                // Move backward
                regDrive(-100,-100,1);
                // Open servo to place purple pixel
                servo.setPosition();
                // Move backward
                regDrive(100,100,0.5);
                // Wait 1 second
                sleep(1000);
                // Close servo
                claw.setPosition();
                // Turn 90 degrees
                robot.rightTurn(0.6);
                // Move forward
                regDrive(100,100,0.5);
                // Extend viperslides slightly and slowly
                VPmove(100,100,0.5);
                // Move armServo and armServo2 to a 60-degree angle
                armServo.setPosition(60.0 / 180.0);
                armServo2.setPosition(60.0 / 180.0);
                // Wait 1 second
                sleep(1000);
                // Open claw2 servo
                claw2.setPosition(0.7);
                // Move back a little
                regDrive(100,100,1);
                // Close claw2 and move arm back to normal
                claw2.setPosition(0.2);
                armServo.setPosition(0.5);
                armServo2.setPosition(0.5);
                // Retract viperslides
                VPmove(0,0,0.5);
                // Strafe to the right a bit
                specDrive(0,0,0,0,0.5);
                // Move forward a little
                regDrive(100,100,0.5);
                break;
*/
            } else if (position == PropPosition.RIGHT) {
                telemetry.addData("Prop Position", "RIGHT");
                telemetry.update();
/*
                // Reset arm servos and claw servos
                armServo.setPosition();
                claw.setPosition();
                claw2.setPosition();
                // Move forward a little
                regDrive(100,100,0.5);
                // Turn right 90 degrees
                robot.rightTurn(0.6);
                // Move forward
                regDrive(100,100,0.5);
                // Move backward
                regDrive(100,100,0.5);
                // Open servo to place pixel on spike mark
                claw.setPosition();
                // Move backward
                regDrive(100,100,0.5);
                // Wait 1 second
                sleep(1000);
                // Close servo
                claw.setPosition();
                // Strafe right
                specDrive(0,0,0,0,0.5);
                // Move forward
                regDrive(100,100,0.5);
                // Extend viperslides slightly and slowly
                VPmove(100,100,0.5);
                // Move armServo and armServo2 to a 60-degree angle
                armServo.setPosition(60.0 / 180.0);
                armServo2.setPosition(60.0 / 180.0);
                // Wait 1 second
                sleep(1000);
                // Open claw2 servo
                claw2.setPosition(0.7);
                // Move back a little
                regDrive(100,100,1);
                // Close claw2 and move arm back to normal
                claw2.setPosition(0.2);
                armServo.setPosition(0.5);
                armServo2.setPosition(0.5);
                // Retract viperslides
                VPmove(0,0,0.5);
                // Strafe to the right a bit
                specDrive(0,0,0,0,0.5);
                // Move forward a little
                regDrive(100,100,0.5);
                break;
*/
            } else {
                telemetry.addData("Prop Position", "ERROR");
                telemetry.update();
/*

*/
            }
        }
    }

    private void regDrive(int leftAim, int rightAim, double speed) {
        leftFrontPos += leftAim;
        leftBackPos += leftAim;
        rightFrontPos += rightAim;
        rightBackPos += rightAim;

        leftFront.setTargetPosition(leftAim);
        leftBack.setTargetPosition(leftAim);
        rightFront.setTargetPosition(rightAim);
        rightBack.setTargetPosition(rightAim);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while(opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            idle();
        }
    }

    private void specDrive(int leftFrontAim, int leftBackAim, int rightFrontAim, int rightBackAim, double speed) {
        leftFrontPos += leftFrontAim;
        leftBackPos += leftBackAim;
        rightFrontPos += rightFrontAim;
        rightBackPos += rightBackAim;

        leftFront.setTargetPosition(leftFrontAim);
        leftBack.setTargetPosition(leftBackAim);
        rightFront.setTargetPosition(rightFrontAim);
        rightBack.setTargetPosition(rightBackAim);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);

        while(opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            idle();
        }
    }

    private void VPmove(int leftAim, int rightAim, double speed) {
        leftVPPos += leftAim;
        rightVPPos += rightAim;

        leftVP.setTargetPosition(leftAim);
        rightVP.setTargetPosition(rightAim);

        leftVP.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVP.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftVP.setPower(speed);
        rightVP.setPower(speed);

        while(opModeIsActive() && leftVP.isBusy() && rightVP.isBusy()) {
            idle();
        }
    }
}