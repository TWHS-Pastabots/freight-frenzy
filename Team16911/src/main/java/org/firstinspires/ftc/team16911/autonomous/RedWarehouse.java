package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@Autonomous(name = "RedWarehouse")
public class RedWarehouse extends LinearOpMode
{
    private SampleMecanumDrive drive;

    private int initialWaitTime = 0;
    private final int[] positions = {120, 170, 220};

    private final Pose2d barcode = new Pose2d(20, .5, 0);
    private final Pose2d hubLevelOnePose = new Pose2d(16, 18.75, 0);
    private final Pose2d hubLevelTwoPose = new Pose2d(16.875, 18.75, 0);
    private final Pose2d hubLevelThreePose = new Pose2d(23, 18.75, 0);
    private final Pose2d warehouseOutside = new Pose2d(0, -27, 0);
    private final Pose2d warehouse = new Pose2d(-.25, -32, 0);

    private Trajectory toBarcode;
    private final Trajectory[] toHubTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubTrajectories = new Trajectory[3];


    public void runOpMode()
    {
        // Configuration Variables
        final int startPosition = 60;

        // Initialize Hardware
        RigatoniHardware hardware = new RigatoniHardware();
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

        utilities.wait(initialWaitTime, telemetry);
        utilities.moveArm(positions[1]);

        drive.followTrajectory(toBarcode);
        int barcodeLevel = utilities.getBarcodeLevelRedSide();
        utilities.moveArm(positions[barcodeLevel]);
        telemetry.addData("Right Distance", hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left Distance", hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        drive.followTrajectory(toHubTrajectories[barcodeLevel]);
        utilities.eliminateOscillations();
        utilities.dropCargo(3500, telemetry);

        drive.followTrajectory(fromHubTrajectories[barcodeLevel]);
    }

    private void buildTrajectories()
    {
        // Configuration Variables
        Trajectory toHubLevelTwo, toHubLevelThree, fromHubLevelOne, fromHubLevelTwo;
        Trajectory fromHubLevelThree, toHubLevelOne;

        toBarcode = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(barcode).build();

        toHubLevelOne = drive.trajectoryBuilder(toBarcode.end())
                .lineToLinearHeading(hubLevelOnePose).build();

        toHubLevelTwo = drive.trajectoryBuilder(toBarcode.end())
                .lineToLinearHeading(hubLevelTwoPose).build();

        toHubLevelThree = drive.trajectoryBuilder(toBarcode.end())
                .lineToLinearHeading(hubLevelThreePose).build();

        fromHubLevelOne = drive.trajectoryBuilder(toHubLevelOne.end(), -Math.toRadians(135))
                .splineToLinearHeading(warehouseOutside, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        fromHubLevelTwo = drive.trajectoryBuilder(toHubLevelTwo.end(), -Math.toRadians(140))
                .splineToLinearHeading(warehouseOutside, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        fromHubLevelThree = drive.trajectoryBuilder(toHubLevelThree.end(), -Math.toRadians(140))
                .splineToLinearHeading(warehouseOutside, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

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
        int lockoutTime = 200;

        while (!isStarted() && !gamepad1.cross)
        {
            if (gamepad1.dpad_up && buttonTime.time() > lockoutTime)
            {
                initialWaitTime = Math.min(10000, initialWaitTime + 1000);
                buttonTime.reset();
            }
            else if (gamepad1.dpad_down && buttonTime.time() > lockoutTime)
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
}
