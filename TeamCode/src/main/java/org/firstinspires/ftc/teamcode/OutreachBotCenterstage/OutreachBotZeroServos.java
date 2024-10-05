package org.firstinspires.ftc.teamcode.OutreachBotCenterstage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class OutreachBotZeroServos extends OpMode {

    Servo claw;
    Servo armServo;

    @Override
    public void init() {

        arm = hardwareMap.get(Servo.class, "claw");
        clawServo = hardwareMap.get(Servo.class, "armServo");

        arm.setPosition(0);
        clawServo.setPosition(0);

    }

    @Override
    public void loop() {

    }
}
