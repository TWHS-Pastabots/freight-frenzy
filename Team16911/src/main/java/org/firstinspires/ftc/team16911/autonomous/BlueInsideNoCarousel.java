package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team16911.R;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;


@Autonomous(name = "BlueInsideNoCarousel")
public class BlueInsideNoCarousel extends LinearOpMode
{
    private RigatoniHardware hardware;
    private SampleMecanumDrive drive;

    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector objectDetector;

    private static final double MIN_CONFIDENCE = .7;
    private static final String ASSET_NAME = null;
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";

    private int maxPosition = 220, startPosition = 35, initialWaitTime = 0;

    private Pose2d firstPosition = new Pose2d(24, -17, 0);
    private Pose2d secondPosition = new Pose2d(0,0, 0);
    private Pose2d thirdPosition = new Pose2d(0, 32, 0);

    private Trajectory firstTrajectory, secondTrajectory, thirdTrajectory;

    public void runOpMode()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        moveArm(startPosition);
        buildTrajectories();

        configuration();

        waitForStart();
        if(!opModeIsActive()) {return;}

        wait(initialWaitTime);

        moveArm(maxPosition);
        drive.followTrajectory(firstTrajectory);
        dropCargo(2000);

        drive.followTrajectory(secondTrajectory);
        drive.followTrajectory(thirdTrajectory);
    }

    private void buildTrajectories()
    {
        firstTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(firstPosition).build();

        secondTrajectory = drive.trajectoryBuilder(firstTrajectory.end())
                .lineToLinearHeading(secondPosition).build();

        thirdTrajectory = drive.trajectoryBuilder(secondTrajectory.end())
                .lineToLinearHeading(thirdPosition).build();
    }

    private void moveArm(int position)
    {
        hardware.armMotorOne.setTargetPosition(position);
        hardware.armMotorTwo.setTargetPosition(position);

        hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        hardware.armMotorOne.setPower(.8);
        hardware.armMotorTwo.setPower(.8);
    }

    private void dropCargo(int waitTime)
    {
        hardware.armServo.setPower(-1);
        wait(waitTime);
        hardware.armServo.setPower(0);
    }

    private void wait(int waitTime)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time() < waitTime)
        {
            continue;
        }
    }

    private void configuration()
    {
        ElapsedTime buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (!gamepad1.x)
        {
            if (isStarted())
            {
                break;
            }
            else if (gamepad1.dpad_up && buttonTime.time() > 300)
            {
                initialWaitTime = Math.min(10000, initialWaitTime + 1000);
                buttonTime.reset();
            }
            else if (gamepad1.dpad_down && buttonTime.time() > 300)
            {
                initialWaitTime = Math.max(0, initialWaitTime - 1000);
                buttonTime.reset();
            }
            else if (gamepad1.circle)
            {
                initialWaitTime = 0;
            }

            telemetry.addData("Initial Wait Time", initialWaitTime / 1000);
            telemetry.update();
        }

        telemetry.addData("Status", "Confirmed");
        telemetry.update();
    }

    private void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = null;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) MIN_CONFIDENCE;
        objectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforiaLocalizer);
        objectDetector.loadModelFromAsset(ASSET_NAME, QUAD_LABEL, SINGLE_LABEL);
        //dashboard.startCameraStream(tfod, 10);
    }

    private void activateTfod()
    {
        // Initialize Vuforia and TFOD
        initVuforia();
        initTfod();

        // Activate TFOD if it can be activated
        if (objectDetector != null) {
            objectDetector.activate();

            objectDetector.setZoom(1.25, 16.0 / 9.0);
        }
    }
}
