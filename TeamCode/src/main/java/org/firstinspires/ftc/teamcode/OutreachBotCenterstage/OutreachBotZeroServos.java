package org.firstinspires.ftc.teamcode.OutreachBotCenterstage;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class OutreachBotZeroServos extends OpMode {

    Servo arm;
    Servo clawServo;

    @Override
    public void init() {

        arm = hardwareMap.get(Servo.class, "arm");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        arm.setPosition(0);
        clawServo.setPosition(0);

    }

    @Override
    public void loop() {

    }
}
