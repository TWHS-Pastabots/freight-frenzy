package org.firstinspires.ftc.team16909.autonomousv2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team16909.autonomousv2.blocks.RedBlocks;
import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(preselectTeleOp = "FettuccineRRv2")
public class RedAuton extends LinearOpMode {

    FettuccineHardware robot = null;
    SampleMecanumDrive drive = null;
    Actions actions = null;

    private Pose2d startPos = null;
    private Pose2d initPos = null;

    private double delay = 0;

    // "Ok" Button
    private int ok = 0;
    private int ok2 = 0;

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private double crThreshHigh = 170;
    private double crThreshLow = 0;
    private double cbThreshHigh = 170;
    private double cbThreshLow = 0;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.9; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    private int camLevel = 1;

    // Green Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 170.0);

    // LISTS
    private ArrayList<String> startChoices = new ArrayList<String>() {{
       add("RedC");
       add("RedW");
    }};
    private ArrayList<String> blockChoices = new ArrayList<String>() {{
        add("RedPushbackC");
        add("RedPushbackW");
        add("RedHub");
        add("RedCarousel");
        add("RedUnit");
        add("RedWarehouse");
        add("Finish");
    }};

    private ArrayList<String> sequence = new ArrayList<String>();


    public void runOpMode() throws InterruptedException {

        robot = new FettuccineHardware();
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actions = new Actions(robot);

        RedBlocks redBlocks = new RedBlocks(drive, robot, actions);

        while (ok == 0) {
            telemetry.addData("Delay", delay);
            if (gamepad1.dpad_right) {
                if (delay < 10) delay++;
                waitFor(0.1);
            } else if (gamepad1.dpad_left) {
                if (delay > 0) delay--;
                waitFor(0.1);
            } else if (gamepad1.dpad_up) ok = 1;
            telemetry.update();
        }
        ok = 0;
        telemetry.addData("Added Delay", delay);
        telemetry.update();
        waitFor(1);

       int i = 0;
       while (ok == 0) {
           telemetry.addData("Start", startChoices.get(i));
           if (gamepad1.dpad_right) {
               if (i < startChoices.size() - 1) i++;
               else i = 0;
               waitFor(0.1);
           } else if (gamepad1.dpad_left) {
               if (i>0) i--;
               else i = startChoices.size() - 1;
               waitFor(0.1);
           } else if (gamepad1.dpad_up) ok = 1;
           telemetry.update();
       }
       ok = 0;
       startPos = redBlocks.getStart(startChoices.get(i));
       telemetry.addData("Added Start", startChoices.get(i));
       telemetry.update();
       waitFor(1);

        i = 0;
        while (ok == 0) {
            while (ok2 == 0) {
                telemetry.addData("Selected", blockChoices.get(i));
                if (gamepad1.dpad_right) {
                    if (i < blockChoices.size() - 1) i++;
                    else i = 0;
                    waitFor(0.1);
                } else if (gamepad1.dpad_left) {
                    if (i > 0) i--;
                    else i = blockChoices.size() - 1;
                    waitFor(0.1);
                } else if (gamepad1.dpad_up) ok2 = 1;
                for (int n = 0; n < sequence.size(); n++) telemetry.addLine(sequence.get(n));
                telemetry.update();
            }
            if (blockChoices.get(i).equals("Finish")) ok = 1;
            else sequence.add(blockChoices.get(i));
            telemetry.addData("Added", blockChoices.get(i));
            telemetry.update();
            i = 0;
            ok2 = 0;
            waitFor(1);
        }
        ok = 0;
        waitFor(0.1);
        telemetry.addLine("Waiting for Start.");
        for (int n = 0; n < sequence.size(); n++) telemetry.addLine(sequence.get(n));
        telemetry.update();

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0.0, 0.0, 0.4, 0.0);
        //previously all 0.2

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        ElapsedTime totalTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (totalTime.seconds() < 2)
        {
            if(pipeline.error){
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            // testing(pipeline);

            // Watch our YouTube Tutorial for the better explanation

            double rectangleArea = pipeline.getRectArea();

            //Print out the area of the rectangle that is found.
            telemetry.addData("Rectangle Area", rectangleArea);

            //Check to see if the rectangle has a large enough area to be a marker.
//            if(rectangleArea > minRectangleArea){
//                //Then check the location of the rectangle to see which barcode it is in.
//                if(pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()){
//                    telemetry.addData("Barcode Position", "Right");
//                }
//                else if(pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()){
//                    telemetry.addData("Barcode Position", "Left");
//                }
//                else {
//                    telemetry.addData("Barcode Position", "Center");
//                }
//            }

            if (pipeline.getRectMidpointX() > 140 && pipeline.getRectMidpointX() < 220) camLevel = 1;
            else if (pipeline.getRectMidpointX() > 220 && pipeline.getRectMidpointX() < 550) camLevel = 2;
            else camLevel = 3;
            telemetry.addData("Level", camLevel);
            telemetry.update();
        }

        waitFor(delay);

        String current = null;
        drive.setPoseEstimate(startPos);
        for (int n = 0; n < sequence.size(); n++) {
            telemetry.addData("pos", camLevel);
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("Block", sequence.get(n));
            telemetry.update();
            current = sequence.get(n);
            if (current.equals("RedPushbackC")) initPos = redBlocks.RedPushbackC(startPos);
            else if (current.equals("RedPushbackW")) initPos = redBlocks.RedPushbackW(startPos);
            else if (current.equals("RedHub")) initPos = redBlocks.RedHub(initPos, camLevel, 1);
            else if (current.equals("RedCarousel")) initPos = redBlocks.RedCarousel(initPos);
            else if (current.equals("RedUnit")) initPos = redBlocks.RedUnit(initPos);
            else if (current.equals("RedWarehouse")) initPos = redBlocks.RedWarehouse(initPos);


        }


    }

    public void waitFor (double time) {

        ElapsedTime duration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = duration.seconds();

        while (duration.seconds() - startTime < time) {

        }
        return;
    }
}
