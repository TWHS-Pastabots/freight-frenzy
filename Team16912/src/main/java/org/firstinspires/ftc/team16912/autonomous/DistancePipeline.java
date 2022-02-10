package org.firstinspires.ftc.team16912.autonomous;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

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

    public double DISTANCE;
    public ArrayList<Block> BLOCKS = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        BLOCKS.clear();
        Mat edges = new Mat(input.rows(), input.cols(), input.type());
        Mat mask = new Mat(input.rows(), input.cols(), input.type());
        Mat YcrCb = new Mat(input.rows(), input.cols(), input.type());

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();


        Imgproc.cvtColor(input, YcrCb, Imgproc.COLOR_BGR2YCrCb);
        Core.inRange(YcrCb, new Scalar(0, 0, 0), new Scalar(255, 255, 150), mask);

        Core.bitwise_not(mask, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(5, 5), 0);
        Imgproc.Canny(mask, edges, 20, 105);

        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            MatOfPoint2f cont = new MatOfPoint2f(contour.toArray());
            double area = Imgproc.contourArea(contour);
            if (area > 500) {
                double peri = Imgproc.arcLength(cont, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(cont, approx, .02 * peri, true);
                Point[] points = approx.toArray();

                List<MatOfPoint> conts = new ArrayList<>();
                conts.add(contour);
                Rect rect = Imgproc.boundingRect(approx);
                Imgproc.drawContours(input, conts, -1, new Scalar(0, 255, 0), 2);
                BLOCKS.add(new Block(rect, (double) (2 * 420) / rect.height, area));
            }

        }
        return input;
    }

    public Block getClosestBlock()
    {
        Block closest = new Block();
        for(Block b : BLOCKS)
        {
            if(b.dist < closest.dist) closest = b;
        }
        return closest;

    }

}
