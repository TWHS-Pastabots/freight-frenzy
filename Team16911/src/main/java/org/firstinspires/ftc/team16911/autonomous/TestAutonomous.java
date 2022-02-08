package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.drive.DriveConstants;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends LinearOpMode
{
    private SampleMecanumDrive drive;

    private int initialWaitTime = 0;
    private double blockPickupPositionY = -42.5;
    private double blockPickupPositionX = 7;
    private double returnSetupPositionX = 4;
    private boolean goForNewBlock = true;

    private final Pose2d barcode = new Pose2d(20, .5, 0);
    private final Pose2d hubLevelOnePose = new Pose2d(16.8, 19.25, 0);
    private final Pose2d hubLevelTwoPose = new Pose2d(17, 19.25, 0);
    private final Pose2d hubLevelThreePose = new Pose2d(23, 19.25, 0);
    private final Pose2d warehouseOutside = new Pose2d(-.25, -20, 0);
    private final Pose2d warehouse = new Pose2d(-.25, -32, 0);

    private Trajectory toBarcode;
    private Utilities utilities;
    private final Trajectory[] toHubTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubTrajectories = new Trajectory[3];


    public void runOpMode() {
        // Configuration
        final int startPosition = 60;
        RigatoniHardware hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        utilities = new Utilities(hardware);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        utilities.moveArm(startPosition);
        buildTrajectories();

        configuration();

        waitForStart();
        if (!opModeIsActive())
        {
            return;
        }

        utilities.wait(initialWaitTime, telemetry);
        utilities.moveArm(utilities.positions[1]);

        drive.followTrajectory(toBarcode);
        int barcodeLevel = utilities.getBarcodeLevelRedSide();
        utilities.moveArm(utilities.positions[barcodeLevel]);
        telemetry.addData("Right Distance", hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left Distance", hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        drive.followTrajectory(toHubTrajectories[barcodeLevel]);
        utilities.eliminateOscillations();
        utilities.dropCargo(utilities.CARGO_DROP_TIME, telemetry);

        drive.followTrajectory(fromHubTrajectories[barcodeLevel]);

        if (goForNewBlock)
        {
            pickupNewBlock();
        }


        /*while (30 - autonomousTime.time() > 10)
        {
            pickupNewBlock();
            blockPickupPositionY -= 2;
            blockPickupPositionX -= 2;
            returnSetupPosition -= 2;
        }*/
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

    private void pickupNewBlock()
    {
        final Pose2d blockPickupOne = new Pose2d(7, -37, Math.toRadians(-90));
        final Pose2d blockPickupTwo = new Pose2d(blockPickupPositionX, blockPickupPositionY - 3, Math.toRadians(-90));
        final Pose2d blockPickupThree = new Pose2d(blockPickupPositionX, blockPickupPositionY, Math.toRadians(-90));
        final Pose2d returnTrajectorySetup = new Pose2d(returnSetupPositionX, -33, 0);
        final Pose2d toHub = new Pose2d(20, 19.25, 0);

        drive.update();

        utilities.moveArm(10);
        utilities.intakeCargo();

        Trajectory blockPickupTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(0))
                .splineToSplineHeading(blockPickupOne, Math.toRadians(-90))
                .splineToSplineHeading(blockPickupTwo, Math.toRadians(-90))
                .splineToSplineHeading(
                        blockPickupThree, Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)).build();

        drive.followTrajectory(blockPickupTrajectory);
        drive.update();

        utilities.stopIntake();
        utilities.moveArm(utilities.positions[2]);
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
            else if (gamepad1.dpad_right)
            {
                goForNewBlock = true;
            }
            else if (gamepad1.dpad_left)
            {
                goForNewBlock = false;
            }

            telemetry.addData("Initial Wait Time", initialWaitTime / 1000);
            telemetry.addData("Go For New BLock", goForNewBlock);
            telemetry.update();
        }

        telemetry.addData("Status", "Confirmed");
        telemetry.update();
    }
}
