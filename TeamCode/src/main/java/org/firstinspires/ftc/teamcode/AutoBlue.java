package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware .DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueTopLeft")
public class AutoBlue extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightVP;
    private DcMotor leftVP;
    private Servo servo1;
    private CRServo servo2;
    private int leftFrontPos;
    private int leftBackPos;
    private int rightFrontPos;
    private int rightBackPos;

    /* Declare OpMode members. */
    RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();
    PropDetection propDetection = new PropDetection(telemetry, true);

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//        servo1 = hardwareMap.get(CRServo.class, "servo1");
//        servo2 = hardwareMap.get(CRServo.class, "servo2");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontPos = 0;
        leftBackPos = 0;
        rightFrontPos = 0;
        rightBackPos = 0;

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

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            PropPosition position = propDetection.getPosition();
            if (position == PropPosition.LEFT) {
                d
            } else if (position == PropPosition.MIDDLE) {
                d
            } else if (position == PropPosition.RIGHT) {
                d
            } else {
                d
            }
//            servo1.setPosition(0.5); // 0-1 range unless changed using scaleRange
//            sleep(x); x being amount of time in milliseconds, used for stops in between servo movement
//            servo2.setPower(1); // 0-1 range
//            regDrive(1000,1000,1,1000);
//            robot.rightTurn(0.6);
//            specificDrive(1000,0,0,1000,1,1000);
//            specificDrive(0,1000,1000,0,1,1000);
            // these control movement and we have to test what distances to move
        }
    }

    private void regDrive(int leftAim, int rightAim, double speed, int time) {
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

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= time) {}
    }

    private void specificDrive(int leftFrontAim, int leftBackAim, int rightFrontAim, int rightBackAim, double speed, int time) {
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

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= time) {}

        while(opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            idle();
        }
    }

    private void VPmove(int leftAim, int rightAim, double speed) {
        leftVP.setTargetPosition(leftAim);
        rightVP.setTargetPosition(rightAim);

        leftVP.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVP.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftVP.setPower(speed);
        rightVP.setPower(speed);
    }
}