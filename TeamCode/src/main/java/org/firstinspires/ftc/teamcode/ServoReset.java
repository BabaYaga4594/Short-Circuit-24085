package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "ServoReset")
public class ServoReset extends LinearOpMode {
    private Servo claw;
    private Servo claw2;
    private Servo armServo;
    private Servo armServo2;
    private Servo drone;

    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        armServo = hardwareMap.get(Servo.class, "armServo");
        armServo2 = hardwareMap.get(Servo.class, "armServo2");
        drone = hardwareMap.get(Servo.class, "drone");

        claw.setDirection(Servo.Direction.REVERSE);
        claw2.setDirection(Servo.Direction.FORWARD);
        armServo.setDirection(Servo.Direction.FORWARD);
        armServo2.setDirection(Servo.Direction.REVERSE);
        drone.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            // Standard pos for "claw"
            claw.setPosition(1.0);
            // Standard pos for "claw2"
            claw2.setPosition(0.0);
            armServo.setPosition(0.0);
            armServo2.setPosition(0.0);
            drone.setPosition(0.0);
        }
    }
}
