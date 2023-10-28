package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "test")
public class TestTeleOp extends OpMode {
    DcMotor frontRight;

    public void moveDriveTrain(){
    }

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    }



    @Override
    public void loop() {
        frontRight.setPower(gamepad1.left_stick_y);
    }
}
