package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.firstinspires.ftc.team16912.util.Util;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Camera Distance Reader")
public class CameraDistanceReader extends LinearOpMode {

    DistancePipeline pipeline;
    OpenCvInternalCamera webcam;

    LinguineHardware robot = new LinguineHardware();
    SampleMecanumDrive drive;

    public void runOpMode() {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new DistancePipeline();
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        if(isStarted())
        {

            centerRobot();

            Trajectory runToBlock = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(pipeline.DISTANCE - 6)
                    .build();

            Util.openClaw(.5);
            drive.followTrajectory(runToBlock);
            Util.runArmTo(170);
            Util.closeClaw(.5);
            Util.runArmTo(-1000);
        }

    }

    private void centerRobot()
    {
        Rect block = pipeline.BLOCKS.get(0).rect;
        int blockCenter = block.x + (block.width/2);
        double pxpi = block.height/2.0;
        double distToEdge = 0.0;
        double strafeDistance = 0.0;

        Trajectory alignBlock;

        // Block is to the left
        if(blockCenter < 160)
        {
            distToEdge = block.x;
            strafeDistance = distToEdge/pxpi + getError(distToEdge, pipeline.DISTANCE);

            alignBlock = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeLeft((strafeDistance)+1)
                    .build();
        }

        // Block is to the right
        else if (blockCenter > 160) {
            distToEdge = 320 - (block.x + block.width);
            strafeDistance = (distToEdge) / pxpi;
            alignBlock = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight((strafeDistance)-1)
                    .build();
        }

        else {
            alignBlock = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(0)
                    .build();
        }

        telemetry.addData("Strafe Distance:", strafeDistance);
        telemetry.addData("Error:", getError(distToEdge, pipeline.DISTANCE));
        telemetry.addData("Distance: ", pipeline.DISTANCE);
        telemetry.addData("Distane to Edge: ", distToEdge);
        telemetry.update();
        drive.followTrajectory(alignBlock);

    }

    private double getError(double distToEdge, double distToBlock)
    {
        double ycomp = 1.125 * Math.pow(1.0675, distToBlock);
        double xcomp;

        double xtop = (Math.pow((distToEdge - 160), 2));

        Rect block = pipeline.BLOCKS.get(0).rect;
        int blockCenter = block.x + (block.width/2);

        // Left
        if (blockCenter < 160) {
            xcomp = xtop / 25000;
        }

        // Right
        else if (blockCenter > 160) {
            xcomp = xtop / 17750;
        }

        else xcomp = 0;

        return ycomp * xcomp;
    }


}
