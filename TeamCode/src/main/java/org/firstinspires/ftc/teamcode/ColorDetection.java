package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetection extends OpenCvPipeline {
    Mat hsvMat = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGBA2RGB); // Only needed if our camera uses BGR instead of RGB otherwise remove line
        Imgproc.cvtColor(hsvMat, hsvMat, Imgproc.COLOR_RGB2HSV); // If line is removed change this to (input, hsvMat, Imgproc.COLOR.RGB2HSV)

        Scalar lower_blue = new Scalar(241, 50, 40);
        Scalar upper_blue = new Scalar(300, 255, 255);
        Scalar lower_red = new Scalar(0, 50, 40);
        Scalar upper_red = new Scalar(60, 255, 255);


        Core.inRange(hsvMat, lower_red, upper_red, hsvMat);
        Core.inRange(hsvMat, lower_blue, upper_blue, hsvMat);

        return null;
    }
}
