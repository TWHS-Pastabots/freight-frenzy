package org.firstinspires.ftc.team16909.autonomous;

import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.SimpleBlobDetector;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorPipeline extends OpenCvPipeline {

    MatOfKeyPoint keypoints;
    Mat result = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        SimpleBlobDetector sbd = SimpleBlobDetector.create();
        sbd.detect(input, keypoints);
        Features2d.drawKeypoints(input, keypoints, result, new Scalar(2,254,255), Features2d.DrawMatchesFlags_DRAW_RICH_KEYPOINTS);
        return result;


    }
}
