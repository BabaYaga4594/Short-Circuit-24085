package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class AutoVP extends LinearOpMode {
    private DcMotor leftVP;
    private DcMotor rightVP;

    @Override
    public void runOpMode() {
        leftVP = hardwareMap.get(DcMotor.class, "viperLeft");
        rightVP = hardwareMap.get(DcMotor.class, "viperRight");
        leftVP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftVP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftVP.setDirection(DcMotor.Direction.REVERSE);
        rightVP.setDirection(DcMotor.Direction.FORWARD);
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