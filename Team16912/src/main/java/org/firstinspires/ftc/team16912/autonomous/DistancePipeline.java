package org.firstinspires.ftc.team16912.autonomous;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

@Config
public class DistancePipeline extends OpenCvPipeline
{
    public static double lower_h = 19.0;
    public static double lower_s = 0.0;
    public static double lower_v = 0.0;

    public static double upper_h = 30.0;
    public static double upper_s = 1000.0;
    public static double upper_v = 1000.0;


    static final Scalar BLUE = new Scalar(0, 0, 255);

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
        Mat edges = new Mat(input.rows(), input.cols(), input.type());
        Mat gray = new Mat(input.rows(), input.cols(), input.type());

        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(gray, edges, new Size(5, 5), 0);
        Imgproc.Canny(edges, edges, 20, 105);

        return edges;
    }

    public int determineDistance() {
        int dis = 0;

        return dis;
    }
}
