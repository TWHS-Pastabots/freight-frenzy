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

import java.security.spec.PSSParameterSpec;


@Autonomous(name = "Test Autonomous")
public class TestAutonomous extends LinearOpMode
{
    private RigatoniHardware hardware;
    private SampleMecanumDrive drive;

    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector objectDetector;

    private static final double MIN_CONFIDENCE = .7;
    private static final String ASSET_NAME = null;
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";

    private int maxPosition = 230;

    private Pose2d firstPosition = new Pose2d(7.25, -23, 6.1116);
    private Pose2d secondPosition = new Pose2d(26.75,28, 0.108);
    private Pose2d thirdPosition = new Pose2d(-10, 50, 0);
    private Pose2d fourthPosition = new Pose2d(-30, 110, 0);

    private Trajectory firstTrajectory, secondTrajectory, thirdTrajectory, fourthTrajectory;

    public void runOpMode()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        buildTrajectories();


        waitForStart();
        if(!opModeIsActive()) {return;}

        drive.followTrajectory(firstTrajectory);
        SpinCarouselAndMoveArm();

        drive.followTrajectory(secondTrajectory);
        DropCargo();

        drive.followTrajectory(thirdTrajectory);
        drive.followTrajectory(fourthTrajectory);
    }

    private void buildTrajectories()
    {
        firstTrajectory = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(firstPosition).build();

        secondTrajectory = drive.trajectoryBuilder(firstTrajectory.end())
                .lineToLinearHeading(secondPosition).build();

        thirdTrajectory = drive.trajectoryBuilder(secondTrajectory.end())
                .lineToLinearHeading(thirdPosition).build();

        fourthTrajectory = drive.trajectoryBuilder(thirdTrajectory.end())
                .lineToLinearHeading(fourthPosition).build();

    }

    private void SpinCarouselAndMoveArm()
    {
        hardware.carouselMotorOne.setPower(.6);
        hardware.carouselMotorTwo.setPower(.6);
        MoveArm();
        wait(2000);
        hardware.carouselMotorOne.setPower(0.0);
        hardware.carouselMotorTwo.setPower(0.0);
    }

    private void MoveArm()
    {
        hardware.armMotorOne.setTargetPosition(maxPosition);
        hardware.armMotorTwo.setTargetPosition(maxPosition);

        hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        hardware.armMotorOne.setPower(.8);
        hardware.armMotorTwo.setPower(.8);
    }

    private void DropCargo()
    {
        hardware.armServo.setPower(-1);
        wait(3000);
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
