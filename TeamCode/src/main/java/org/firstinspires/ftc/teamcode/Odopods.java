package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odopods {

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux;

    public Odopods(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoderLeft = leftBack;
        encoderRight = rightBack;
        encoderAux = rightFront;
        /**
         * Need to change which motors the encoders are connected to based on the ports where the
         * encoders are plugged in on the control/expansion hub
         **/
    }

    // these constants define the geometry of our robot
    final static double L = 20.0;     // distance between left and right encoder in cm
    final static double B = 11.0;     // distance between the midpoint of left and right encoder and aux encoder
    final static double R = 3.0;      // omni-wheel radius in cm
    final static double N = 8192;     // encoder ticks per revolution
    final static double cm_per_tick = 2.0 * Math.PI * R / N;
    /**
     * Need to adjust values of variables to our robot's measurements after mechanical sets up
     * odopods on the robot
    **/

    // keeps track of the odometry encoders between updates
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;

    /**
     * Odometry Notes:
     *
     * n1, n2, n3 are encoder values for the left, right, and aux omni-wheels
     * dn1, dn2, dn3 are the difference between encoder values of two readings
     * dx, dy, dtheta describe the robot movement between two readings (in robot coords)
     * X, Y, Theta are the coordinates ont he field and the heading of the robot
     **/

    // XyhVector is a tuple (x,y,h) where h is the heading of the robot
    public XyhVector START_POS = new XyhVector(213, 102, Math.toRadians(-174));
    public XyhVector pos = new XyhVector(START_POS);

    public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition  - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        // the robot has moved and turned a tiny bit between two measurements:
        double dtheta = cm_per_tick * ((dn2-dn1) / L);
        double dx = cm_per_tick * ((dn1+dn2) / 2.0);
        double dy = cm_per_tick * (dn3 + (dn2-dn1) * B / L );

        // small movement of the robot gets added to the field coordinate system:
        pos.h += dtheta / 2;
        pos.x += dx * Math.cos(pos.h) - dy * Math.sin(pos.h);
        pos.y += dx * Math.sin(pos.h) + dy * Math.cos(pos.h);
        pos.h = pos.normDiff(pos.h);
    }
}
