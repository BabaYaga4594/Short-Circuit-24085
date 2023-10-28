package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servo extends OpMode {
//    public Servo servo;
    public Servo servo1;
    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo");
//        servo = hardwareMap.get(CRServoServo.class, "servo");


    }
    @Override
    public void loop(){
        if(gamepad1.a){
            servo1.setPosition(0);
//            servo.setPower(1);
        }
        if(gamepad1.a){
            servo1.setPosition(1);
//            servo.setPower(-1);
        }
//        servo.setPower(0);
    }
}
