package org.firstinspires.ftc.teamcode.OutreachBotCenterstage;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "OutreachBotMecanumDrive")
public class OutreachBotMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //init
        // different fucntions to look at, drive, turn, strafe
        // drive - use left joystick y, and strafe for left and right on left joystick x
        double drive, turn, strafe;
        double flpower, frpower, blpower, brpower;
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
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
        }
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
