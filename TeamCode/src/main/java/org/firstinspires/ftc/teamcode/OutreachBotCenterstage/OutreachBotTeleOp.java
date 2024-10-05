package org.firstinspires.ftc.teamcode.OutreachBotCenterstage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OutreachBotTeleOp")
public class OutreachBotTeleOp extends OpMode {

    double drive, turn, strafe;
    double flpower, frpower, blpower, brpower;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    //DcMotor leftViperSlide;
    //DcMotor rightViperSlide;

    Servo claw;
    Servo armServo;

    @Override
    public void init() {
        // change the names for all motors look at driver hub config
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //leftViperSlide = hardwareMap.get(DcMotor.class, "left viper slide");
        //rightViperSlide = hardwareMap.get(DcMotor.class, "right viper slide");

        claw = hardwareMap.get(Servo.class, "claw");
        armServo = hardwareMap.get(Servo.class, "armServo");

        // if needed change reversals if it is left side
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightViperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        drive = gamepad1.left_stick_y * -1;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        flpower = drive + turn  + strafe;
        frpower = drive - turn - strafe;
        blpower = drive + turn - strafe;
        brpower = drive - turn + strafe;

        double[] appliedPowers = scalePowers(flpower, frpower, blpower, brpower);

        frontLeft.setPower(appliedPowers[0]);
        frontRight.setPower(appliedPowers[1]);
        backLeft.setPower(appliedPowers[2]);
        backRight.setPower(appliedPowers[3]);
        leftViperSlide.setPower(0.5);
        rightViperSlide.setPower(0.5);


        //if (gamepad1.dpad_up) {
            //leftViperSlide.setTargetPosition(100);
            //rightViperSlide.setTargetPosition(100);
            //leftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //}
        //else if (gamepad1.dpad_down) {
            //leftViperSlide.setTargetPosition(-100);
            //rightViperSlide.setTargetPosition(-100);
            //leftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //}

        // add arm and claw servo motions here

    }

    public double[] scalePowers(double flpower, double frpower, double blpower, double brpower){
        double max = Math.max(Math.abs(flpower), Math.max(Math.abs(frpower), Math.max(Math.abs(blpower),Math.abs(brpower))));
        if(max > 1){
            flpower /= max;
            frpower /= max;
            blpower /= max;
            brpower /= max;
        }

        double[] motorPowers = new double[]{flpower, frpower, blpower, brpower};
        return motorPowers;
    }
}
