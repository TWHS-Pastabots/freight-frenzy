package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "camTest")
public class cameraTest extends LinearOpMode{


    SampleMecanumDrive drive;

    private Pose2d shippingHub = new Pose2d(17.5, 29, 0);
    private Pose2d positionOne = new Pose2d(6.5, -18, Math.toRadians(90));
    private Pose2d positionTwo = new Pose2d(30.5, -21, Math.toRadians(90));

    private Trajectory toShippingHub;
    private Trajectory trajectoryOne;
    private Trajectory trajectoryTwo;

    private final String STORAGE = "Storage";
    private final String NO_STORAGE = "No Storage";
    private String endPoint = STORAGE;

    OpenCvInternalCamera webcam;
    CodeBarPipeline pipeline;



    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new CodeBarPipeline();
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



        RavioliHardware hardware = new RavioliHardware();
        util utilities = new util(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        hardware.init(hardwareMap);

        while (!gamepad1.triangle) {
            telemetry.addData("Left Brightness: ", pipeline.R1Y);
            telemetry.addData("Middle Brightness: ", pipeline.R2Y);
            telemetry.addData("Right Brightness: ", pipeline.R3Y);
            telemetry.update();
        }

        buildTrajectories();
        configuration();


        waitForStart();

        while (isStarted()&&!isStopRequested())
        {

            CodeBarPipeline.ObjectPosition pos = pipeline.getAnalysis();
            telemetry.addData("aaa", "aaaa");
            telemetry.update();

            // No camera yet so robot will always deliver to top for now

        }

    }


    private void buildTrajectories()
    {
        toShippingHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(shippingHub).build();

        trajectoryOne = drive.trajectoryBuilder(toShippingHub.end())
                .lineToLinearHeading(positionOne).build();

        trajectoryTwo = drive.trajectoryBuilder(trajectoryOne.end())
                .lineToLinearHeading(positionTwo).build();
    }
    private void configuration()
    {
        while (!gamepad1.cross && !isStarted())
        {
            if(gamepad1.right_bumper)
            {
                endPoint = STORAGE;
            }
            else if(gamepad1.left_bumper)
            {
                endPoint = NO_STORAGE;
            }
            telemetry.addData("Endpoint", endPoint);
            telemetry.update();
        }
        telemetry.addData("Status", "Confirmed");
        telemetry.update();
    }
}
