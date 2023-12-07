/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {
    private DcMotor leftFront = null;

    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    private Telemetry telemetry;

    public RobotHardware() {}

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        telemetry.addData(">", "Hardware Initialized");
        telemetry.update();
    }

    public void leftTurn(double speed){
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);
    }

    public void rightTurn(double speed){
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);
    }

    public void backward(double speed, long time){
        leftFront.setPower(-speed);
        rightFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(-speed);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= time) {}
    }

    public void forward(double speed, long time) {
        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= time) {}
    }

    public void stop(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void leftStrafe(double speed, long time){
        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(-speed);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= time) {}
    }

    public void rightStrafe(double speed, long time){
        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightBack.setPower(speed);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds() <= time) {}
    }
}
