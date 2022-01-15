package org.firstinspires.ftc.team16912.autonomous;

import static org.opencv.core.Core.inRange;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

public class DistancePipeline extends OpenCvPipeline {


    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(0, 180);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;


    Point region1_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);







    @Override
    public Mat processFrame(Mat input)
    {
        Mat HSV = new Mat();
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        inRange(HSV, new Scalar(28, 10, 10), new Scalar(47, 255, 255), mask);


        input.setTo(Scalar.all(255), mask);
        Mat gray = new Mat(input.rows(), input.cols(), input.type());
        Mat edges = new Mat(input.rows(), input.cols(), input.type());
        Mat dst = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        //Blurring the image
        Imgproc.blur(gray, edges, new Size(3, 3));
        //Detecting the edges
        Imgproc.Canny(edges, edges, 100, 100*3);

        edges.copyTo(input);


        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        return input;
    }

    public int determineDistance() {
        int dis = 0;

        return dis;
    }
}
