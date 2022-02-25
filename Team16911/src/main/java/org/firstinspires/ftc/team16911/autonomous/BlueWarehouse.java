package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.drive.DriveConstants;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@Autonomous(name = "BlueWarehouse")
public class BlueWarehouse extends LinearOpMode
{
    private SampleMecanumDrive drive;

    private int initialWaitTime = 0;
    private boolean attemptFirstBlock = true;
    private boolean attemptSecondBlock = true;

    private final Pose2d barcode = new Pose2d(18.5, -.5, 0);
    private final Pose2d hubLevelOnePose = new Pose2d(16.5, -19.25, 0);
    private final Pose2d hubLevelTwoPose = new Pose2d(17, -19.25, 0);
    private final Pose2d hubLevelThreePose = new Pose2d(23, -19.25, 0);
    private final Pose2d warehouseOutside = new Pose2d(-.25, 20, 0);
    private final Pose2d warehouse = new Pose2d(-.25, 30, 0);

    private double blockPickupPositionY = 40;
    private double blockPickupPositionX = 7;
    private double returnSetupPositionX = 4;

    private Trajectory toBarcode;
    private Utilities utilities;
    private final Trajectory[] toHubTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubTrajectories = new Trajectory[3];


    public void runOpMode()
    {
        RigatoniHardware hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        utilities = new Utilities(hardware);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        buildTrajectories();

        configuration();

        waitForStart();
        if (!opModeIsActive()) { return; }

        ElapsedTime autonomousTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        utilities.wait(initialWaitTime, telemetry);
        utilities.moveArm(utilities.initialArmPosition);

        drive.followTrajectory(toBarcode);
        int barcodeLevel = utilities.getBarcodeLevelBlueSide();
        if (barcodeLevel != 0) { utilities.moveArm(utilities.positions[barcodeLevel]); }
        telemetry.addData("Right Distance", hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left Distance", hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        drive.followTrajectory(toHubTrajectories[barcodeLevel]);
        if (barcodeLevel == 0)
        {
            utilities.moveArm(utilities.positions[barcodeLevel]);
            utilities.wait(750, telemetry);
        }
        utilities.eliminateOscillations();
        utilities.dropCargo(utilities.CARGO_DROP_TIME, utilities.DROP_POWERS[barcodeLevel], telemetry);

        drive.followTrajectory(fromHubTrajectories[barcodeLevel]);

        if (attemptFirstBlock && 30000 - autonomousTime.time() > 10000)
        {
            pickAndDropNewBlock(hardware);
        }

        blockPickupPositionY += 4.5;
        blockPickupPositionX -= 2;
        returnSetupPositionX -= 2;

        if (attemptSecondBlock && 30000 - autonomousTime.time() > 8000)
        {
            pickupNewBlock(hardware);
        }
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

        fromHubLevelOne = drive.trajectoryBuilder(toHubLevelOne.end(), Math.toRadians(135))
                .splineToLinearHeading(warehouseOutside, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        fromHubLevelTwo = drive.trajectoryBuilder(toHubLevelTwo.end(), Math.toRadians(140))
                .splineToLinearHeading(warehouseOutside, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        fromHubLevelThree = drive.trajectoryBuilder(toHubLevelThree.end(), Math.toRadians(140))
                .splineToLinearHeading(warehouseOutside, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        toHubTrajectories[0] = toHubLevelOne;
        toHubTrajectories[1] = toHubLevelTwo;
        toHubTrajectories[2] = toHubLevelThree;

        fromHubTrajectories[0] = fromHubLevelOne;
        fromHubTrajectories[1] = fromHubLevelTwo;
        fromHubTrajectories[2] = fromHubLevelThree;
    }

    private void pickAndDropNewBlock(RigatoniHardware hardware)
    {
        final Pose2d blockPickupOne = new Pose2d(7, 37, Math.toRadians(90));
        final Pose2d blockPickupTwo = new Pose2d(blockPickupPositionX, blockPickupPositionY - 4, Math.toRadians(90));
        final Pose2d blockPickupThree = new Pose2d(blockPickupPositionX, blockPickupPositionY, Math.toRadians(90));
        final Pose2d returnTrajectorySetup = new Pose2d(returnSetupPositionX, 36, 0);
        final Pose2d toHub = new Pose2d(23, -19.25, 0);
        final Vector2d toHubReturn = new Vector2d(-.25, 10);
        final Vector2d warehouse = new Vector2d(-.25, 32);

        drive.update();

        utilities.moveArm(10);
        utilities.wait(750, telemetry);
        utilities.intakeCargo();

        Trajectory blockPickupTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(0))
                .splineToSplineHeading(blockPickupOne, Math.toRadians(90))
                .splineToSplineHeading(blockPickupTwo, Math.toRadians(90))
                .splineToSplineHeading(blockPickupThree, Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(
                                7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

        drive.followTrajectory(blockPickupTrajectory);
        drive.update();

        utilities.stopIntake();
        utilities.moveArm(utilities.positions[2] + 5);

        double distanceOne = hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double distanceTwo =  hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double distanceThree =  hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double distanceFromWall = (distanceOne + distanceTwo + distanceThree) / 3.0;
        double yPose = -distanceFromWall + 59;
        yPose = Math.min(48, Math.abs(yPose));
        if (yPose >= 48 || yPose <= 30)
        {
            yPose = 48;
        }
        drive.setPoseEstimate(new Pose2d(blockPickupPositionX, yPose, Math.toRadians(90)));
        drive.update();

        Trajectory toHubTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(-90))
                .splineToSplineHeading(returnTrajectorySetup, Math.toRadians(-140))
                .splineToLinearHeading(this.warehouse, Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(), Math.toRadians(-90))
                .splineToLinearHeading(toHub, Math.toRadians(-45)).build();

        drive.followTrajectory(toHubTrajectory);
        drive.update();

        distanceOne = hardware.backDistanceSensor.getDistance(DistanceUnit.INCH);
        distanceTwo =  hardware.backDistanceSensor.getDistance(DistanceUnit.INCH);
        distanceThree =  hardware.backDistanceSensor.getDistance(DistanceUnit.INCH);
        double backDistance = (distanceOne + distanceTwo + distanceThree) / 3.0;
        backDistance = backDistance - 1;
        backDistance = Math.min(27, backDistance);
        if (backDistance >= 27 || backDistance <= 5)
        {
            backDistance = 27;
        }
        drive.setPoseEstimate(new Pose2d(backDistance, -20, 0));

        utilities.dropCargo(utilities.CARGO_DROP_TIME + 1700, utilities.DROP_POWERS[1],telemetry);

        Trajectory returnToWarehouse = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(140))
                .splineToConstantHeading(toHubReturn, Math.toRadians(90))
                .splineToConstantHeading(warehouse, Math.toRadians(90)).build();

        drive.followTrajectory(returnToWarehouse);
    }

    private void pickupNewBlock(RigatoniHardware hardware)
    {
        final Pose2d blockPickupOne = new Pose2d(blockPickupPositionX, 37, Math.toRadians(90));
        final Pose2d blockPickupTwo = new Pose2d(blockPickupPositionX, blockPickupPositionY - 4, Math.toRadians(90));
        final Pose2d blockPickupThree = new Pose2d(blockPickupPositionX, blockPickupPositionY, Math.toRadians(90));
        final Pose2d returnTrajectorySetup = new Pose2d(returnSetupPositionX, 36, 0);
        final Pose2d warehouse = new Pose2d(-.25, 36, 0);

        drive.update();

        utilities.moveArm(10);
        utilities.wait(750, telemetry);
        utilities.intakeCargo();

        Trajectory blockPickupTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(0))
                .splineToSplineHeading(blockPickupOne, Math.toRadians(90))
                .splineToSplineHeading(blockPickupTwo, Math.toRadians(90))
                .splineToSplineHeading(blockPickupThree, Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(
                                7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

        drive.followTrajectory(blockPickupTrajectory);
        drive.update();

        utilities.stopIntake();

        double distanceOne = hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double distanceTwo =  hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double distanceThree =  hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double distanceFromWall = (distanceOne + distanceTwo + distanceThree) / 3.0;
        double yPose = -distanceFromWall + 60;
        yPose = Math.min(48, Math.abs(yPose));
        drive.setPoseEstimate(new Pose2d(blockPickupPositionX, yPose, Math.toRadians(90)));
        drive.update();

        Trajectory returnTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(-90))
                .splineToSplineHeading(returnTrajectorySetup, Math.toRadians(-140))
                .splineToLinearHeading(warehouse, Math.toRadians(-90)).build();

        drive.followTrajectory(returnTrajectory);
        drive.update();
    }

    private void configuration()
    {
        ElapsedTime buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        int lockoutTime = 200;

        while (!isStarted() && !gamepad1.cross)
        {
            if (gamepad1.dpad_up && buttonTime.time() > lockoutTime)
            {
                initialWaitTime = Math.min(30000, initialWaitTime + 1000);
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
            else if (gamepad1.square && buttonTime.time() > lockoutTime)
            {
                attemptFirstBlock = !attemptFirstBlock;
                buttonTime.reset();
            }
            else if (gamepad1.triangle && buttonTime.time() > lockoutTime)
            {
                attemptSecondBlock = !attemptSecondBlock;
                buttonTime.reset();
            }

            telemetry.addData("Initial Wait Time", initialWaitTime / 1000);
            telemetry.addData("Attempt First Block", attemptFirstBlock);
            telemetry.addData("Attempt Second Block", attemptSecondBlock);
            telemetry.update();
        }

        telemetry.addData("Status", "Confirmed");
        telemetry.update();
    }
}
