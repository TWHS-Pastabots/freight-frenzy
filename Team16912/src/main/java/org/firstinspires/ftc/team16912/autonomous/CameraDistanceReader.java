package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Camera Distance Reader")
public class CameraDistanceReader extends LinearOpMode {

    int cameraGroundHeight = 65; // Height of the center of the lens from the ground
    int blockHeight = 50; // Height of the freight block

    private int distance;

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

            Trajectory runToBlock = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(pipeline.DISTANCE - 5)
                    .build();

            openClaw();

            drive.followTrajectory(runToBlock);

            runArmToStart();

            closeClaw();

        }

    }
    // Closes claw
    private void closeClaw() { robot.servoClaw.setPosition(.7); }

    // Opens claw
    private void openClaw() { robot.servoClaw.setPosition(-1); }

    // Return arm to start
    private void runArmToStart() {
        while (robot.armEncoder.getCurrentPosition() < 200) {
            for (DcMotorEx motor : robot.motorArms) {
                motor.setPower(.5);
            }
        }
        for (DcMotorEx motor : robot.motorArms) motor.setPower(0);
    }


}
