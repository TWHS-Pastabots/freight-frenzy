package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team16911.R;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@Autonomous(name = "RedInsideDistance")
public class RedInsideDistance extends LinearOpMode
{
    private RigatoniHardware hardware;
    private SampleMecanumDrive drive;

    private VuforiaLocalizer vuforiaLocalizer;
    private TFObjectDetector objectDetector;

    private static final double MIN_CONFIDENCE = .7;
    private static final String ASSET_NAME = null;
    private static final String QUAD_LABEL = "Quad";
    private static final String SINGLE_LABEL = "Single";

    private final int startPosition = 35;
    private int initialWaitTime = 0;
    private final int[] positions = {110, 130, 220};

    private final Pose2d firstPosition = new Pose2d(20, .375, 0);
    private final Pose2d hubLevelOnePose = new Pose2d(13.5, 17, 0);
    private final Pose2d hubLevelTwoPose = new Pose2d(16.3, 17, 0);
    private final Pose2d hubLevelThreePose = new Pose2d(22.4, 17, 0);
    private final Pose2d secondPosition = new Pose2d(0, 0, 0);
    private final Pose2d thirdPosition = new Pose2d(0, -32, 0);

    private Trajectory firstTrajectory, secondTrajectory, toHubLevelOne;
    private Trajectory toHubLevelTwo, toHubLevelThree, fromHubLevelOne, fromHubLevelTwo;
    private Trajectory fromHubLevelThree;
    private final Trajectory[] toHubTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubTrajectories = new Trajectory[3];


    public void runOpMode()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        util utilities = new util(hardware);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        utilities.moveArm(startPosition);
        buildTrajectories();

        configuration();

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.wait(initialWaitTime);

        drive.followTrajectory(firstTrajectory);
        int barcodeLevel = utilities.getBarcodeLevelRedSide();
        utilities.moveArm(positions[barcodeLevel]);
        telemetry.addData("Right Distance", hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left Distance", hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        drive.followTrajectory(toHubTrajectories[barcodeLevel]);
        utilities.eliminateOscillations();
        utilities.dropCargo(2000);

        drive.followTrajectory(fromHubTrajectories[barcodeLevel]);
        drive.followTrajectory(secondTrajectory);
    }

    private void buildTrajectories()
    {

        firstTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(firstPosition).build();

        toHubLevelOne = drive.trajectoryBuilder(firstTrajectory.end())
                .lineToLinearHeading(hubLevelOnePose).build();

        toHubLevelTwo = drive.trajectoryBuilder(firstTrajectory.end())
                .lineToLinearHeading(hubLevelTwoPose).build();

        toHubLevelThree = drive.trajectoryBuilder(firstTrajectory.end())
                .lineToLinearHeading(hubLevelThreePose).build();

        fromHubLevelOne = drive.trajectoryBuilder(toHubLevelOne.end())
                .lineToLinearHeading(secondPosition).build();

        fromHubLevelTwo = drive.trajectoryBuilder(toHubLevelTwo.end())
                .lineToLinearHeading(secondPosition).build();

        fromHubLevelThree = drive.trajectoryBuilder(toHubLevelThree.end())
                .lineToLinearHeading(secondPosition).build();

        secondTrajectory = drive.trajectoryBuilder(secondPosition)
                .lineToLinearHeading(thirdPosition).build();

        toHubTrajectories[0] = toHubLevelOne;
        toHubTrajectories[1] = toHubLevelTwo;
        toHubTrajectories[2] = toHubLevelThree;

        fromHubTrajectories[0] = fromHubLevelOne;
        fromHubTrajectories[1] = fromHubLevelTwo;
        fromHubTrajectories[2] = fromHubLevelThree;
    }

    private void configuration()
    {
        ElapsedTime buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (!gamepad1.x)
        {
            if (isStarted() || gamepad1.x)
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
