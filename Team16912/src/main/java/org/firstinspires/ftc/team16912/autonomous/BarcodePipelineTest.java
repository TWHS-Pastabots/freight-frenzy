package org.firstinspires.ftc.team16912.autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BarcodePipelineTest extends OpenCvPipeline {
    /*
     * An enum to define the object position
     */
    public enum ObjectPosition {
        LEFT, // Deliver Bottom
        CENTER, // Deliver Middle
        RIGHT // Deliver Top
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 180);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(150, 180);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(290, 180);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;


    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_H, region2_H, region3_H;
    Mat HSV = new Mat();
    Mat H = new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile ObjectPosition position = ObjectPosition.LEFT;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToH(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(HSV, H, 0);
    }

    @Override
    public void init(Mat firstFrame) {

        //inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_H = H.submat(new Rect(region1_pointA, region1_pointB));
        region2_H = H.submat(new Rect(region2_pointA, region2_pointB));
        region3_H = H.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {

        inputToH(input);

        avg1 = (int) Core.mean(region1_H).val[0];
        avg2 = (int) Core.mean(region2_H).val[0];
        avg3 = (int) Core.mean(region3_H).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the min of the 3 boxes and channels
         */

        int R1Hue = Math.abs(avg1- 120);
        int R2Hue = Math.abs(avg2- 120);
        int R3Hue = Math.abs(avg3- 120);
        int TargetHue = Math.min(R3Hue, Math.min(R1Hue, R2Hue));

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if (TargetHue == avg1) // Was it from region 1?
        {
            position = ObjectPosition.LEFT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        } else if (TargetHue == avg2) // Was it from region 2?
        {
            position = ObjectPosition.CENTER; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        } else if (TargetHue == avg3) // Was it from region 3?
        {
            position = ObjectPosition.RIGHT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public ObjectPosition getAnalysis() {
        return position;
    }

}
