package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@Autonomous(name = "RedCarousel")
public class RedCarousel extends LinearOpMode
{
    private SampleMecanumDrive drive;

    private static final String WAREHOUSE = "Warehouse";
    private static final String STORAGE_UNIT = "Storage Unit";
    private static final String DIRECT_ROUTE = "Direct";
    private static final String BOTTOM_ROUTE = "Bottom";
    private static final String WAREHOUSE_DIRECT_ROUTE = WAREHOUSE + DIRECT_ROUTE;
    private static final String WAREHOUSE_BOTTOM_ROUTE = WAREHOUSE + BOTTOM_ROUTE;
    private static final String STORAGE_DIRECT_ROUTE = STORAGE_UNIT + DIRECT_ROUTE;
    private static final String STORAGE_BOTTOM_ROUTE = STORAGE_UNIT + BOTTOM_ROUTE;
    private static final String[] WAREHOUSE_ROUTES = {DIRECT_ROUTE, BOTTOM_ROUTE};
    private static final String[] STORAGE_ROUTES = {DIRECT_ROUTE, BOTTOM_ROUTE};
    private static String endPosition = WAREHOUSE;
    private static String route = DIRECT_ROUTE;


    private int initialWaitTime = 0;
    private final int[] positions = {100, 130, 200};

    private final Pose2d carousel = new Pose2d(3.75, 18, 0);
    private final Pose2d barcode = new Pose2d(20,-0.18, 0);
    private final Pose2d hubLevelOne = new Pose2d(15, -27.75, 0);
    private final Pose2d hubLevelTwo = new Pose2d(16.875, -27.75, 0);
    private final Pose2d hubLevelThree = new Pose2d(23, -27.75, 0);
    private final Pose2d warehouseOutside = new Pose2d(0, -56, 0);
    private final Pose2d warehouseBottomPosition = new Pose2d(1.5, -36, 0);
    private final Pose2d warehouse = new Pose2d(-.33, -80, 0);
    private final Pose2d barcodeBottomPositionOne = new Pose2d(15, -5, -Math.toRadians(45));
    private final Pose2d barcodeBottomPositionTwo = new Pose2d(15, 5, -Math.toRadians(45));
    private final Pose2d storageUnit = new Pose2d(30,24, -Math.toRadians(90));

    private Trajectory toCarousel, toBarcode;

    private final Trajectory[] toHubTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubDirectTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubBottomTrajectories = new Trajectory[3];
    private final Trajectory[] toStorageDirectTrajectories = new Trajectory[3];
    private final Trajectory[] toStorageBottomTrajectories = new Trajectory[3];

    public void runOpMode()
    {
        // Configuration Variables
        final int startPosition = 35;

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
        final String path = endPosition + route;

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.wait(initialWaitTime, telemetry);

        drive.followTrajectory(toCarousel);
        utilities.spinCarouselAndMoveArm(2700, positions[1], telemetry);

        drive.followTrajectory(toBarcode);
        int barcodeLevel = utilities.getBarcodeLevelRedSide();
        utilities.moveArm(positions[barcodeLevel]);
        telemetry.addData("Right Distance", hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left Distance", hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        drive.followTrajectory(toHubTrajectories[barcodeLevel]);
        utilities.eliminateOscillations();
        utilities.dropCargo(2000, telemetry);

        switch (path)
        {
            case WAREHOUSE_DIRECT_ROUTE:
                drive.followTrajectory(fromHubDirectTrajectories[barcodeLevel]);
                break;
            case WAREHOUSE_BOTTOM_ROUTE:
                drive.followTrajectory(fromHubBottomTrajectories[barcodeLevel]);
                break;
            case STORAGE_DIRECT_ROUTE:
                drive.followTrajectory(toStorageDirectTrajectories[barcodeLevel]);
                break;
            case STORAGE_BOTTOM_ROUTE:
                drive.followTrajectory(toStorageBottomTrajectories[barcodeLevel]);
                break;
        }
    }

    private void buildTrajectories()
    {
        // Configuration Variables
        Trajectory toHubLevelOne, toHubLevelTwo, toHubLevelThree;
        Trajectory fromHubLevelOneDirect, fromHubLevelTwoDirect, fromHubLevelThreeDirect;
        Trajectory fromHubLevelOneBottom, fromHubLevelTwoBottom, fromHubLevelThreeBottom;
        Trajectory toStorageLevelOneDirect, toStorageLevelTwoDirect, toStorageLevelThreeDirect;
        Trajectory toStorageLevelOneBottom, toStorageLevelTwoBottom, toStorageLevelThreeBottom;

        toCarousel = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(carousel).build();

        toBarcode = drive.trajectoryBuilder(toCarousel.end())
                .lineToLinearHeading(barcode).build();

        toHubLevelOne = drive.trajectoryBuilder(toBarcode.end())
                .lineToLinearHeading(hubLevelOne).build();

        toHubLevelTwo = drive.trajectoryBuilder(toBarcode.end())
                .lineToLinearHeading(hubLevelTwo).build();

        toHubLevelThree = drive.trajectoryBuilder(toBarcode.end())
                .lineToLinearHeading(hubLevelThree).build();

        fromHubLevelOneDirect = drive.trajectoryBuilder(toHubLevelOne.end(), -Math.toRadians(120))
                .splineToLinearHeading(warehouseOutside, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        fromHubLevelTwoDirect = drive.trajectoryBuilder(toHubLevelTwo.end(), -Math.toRadians(130))
                .splineToLinearHeading(warehouseOutside, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        fromHubLevelThreeDirect = drive.trajectoryBuilder(toHubLevelThree.end(), -Math.toRadians(130))
                .splineToLinearHeading(warehouseOutside, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        fromHubLevelOneBottom = drive.trajectoryBuilder(toHubLevelOne.end(), -Math.toRadians(180))
                .splineToLinearHeading(warehouseBottomPosition, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        fromHubLevelTwoBottom = drive.trajectoryBuilder(toHubLevelTwo.end(), -Math.toRadians(180))
                .splineToLinearHeading(warehouseBottomPosition, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        fromHubLevelThreeBottom = drive.trajectoryBuilder(toHubLevelThree.end(), -Math.toRadians(180))
                .splineToLinearHeading(warehouseBottomPosition, -Math.toRadians(90))
                .splineToLinearHeading(warehouse, -Math.toRadians(90)).build();

        toStorageLevelOneDirect = drive.trajectoryBuilder(toHubLevelOne.end())
                .lineToSplineHeading(storageUnit).build();

        toStorageLevelTwoDirect = drive.trajectoryBuilder(toHubLevelTwo.end())
                .lineToSplineHeading(storageUnit).build();

        toStorageLevelThreeDirect = drive.trajectoryBuilder(toHubLevelThree.end())
                .lineToSplineHeading(storageUnit).build();

        toStorageLevelOneBottom = drive.trajectoryBuilder(toHubLevelOne.end(), -Math.toRadians(280))
                .splineToSplineHeading(barcodeBottomPositionOne, -Math.toRadians(270))
                .splineToSplineHeading(barcodeBottomPositionTwo, -Math.toRadians(270))
                .splineToSplineHeading(storageUnit, -Math.toRadians(0)).build();

        toStorageLevelTwoBottom = drive.trajectoryBuilder(toHubLevelTwo.end(), -Math.toRadians(265))
                .splineToSplineHeading(barcodeBottomPositionOne, -Math.toRadians(270))
                .splineToSplineHeading(barcodeBottomPositionTwo, -Math.toRadians(270))
                .splineToSplineHeading(storageUnit, -Math.toRadians(0)).build();

        toStorageLevelThreeBottom = drive.trajectoryBuilder(toHubLevelThree.end(), -Math.toRadians(250))
                .splineToSplineHeading(barcodeBottomPositionOne, -Math.toRadians(270))
                .splineToSplineHeading(barcodeBottomPositionTwo, -Math.toRadians(270))
                .splineToSplineHeading(storageUnit, -Math.toRadians(0)).build();

        toHubTrajectories[0] = toHubLevelOne;
        toHubTrajectories[1] = toHubLevelTwo;
        toHubTrajectories[2] = toHubLevelThree;

        fromHubDirectTrajectories[0] = fromHubLevelOneDirect;
        fromHubDirectTrajectories[1] = fromHubLevelTwoDirect;
        fromHubDirectTrajectories[2] = fromHubLevelThreeDirect;

        fromHubBottomTrajectories[0] = fromHubLevelOneBottom;
        fromHubBottomTrajectories[1] = fromHubLevelTwoBottom;
        fromHubBottomTrajectories[2] = fromHubLevelThreeBottom;

        toStorageDirectTrajectories[0] = toStorageLevelOneDirect;
        toStorageDirectTrajectories[1] = toStorageLevelTwoDirect;
        toStorageDirectTrajectories[2] = toStorageLevelThreeDirect;

        toStorageBottomTrajectories[0] = toStorageLevelOneBottom;
        toStorageBottomTrajectories[1] = toStorageLevelTwoBottom;
        toStorageBottomTrajectories[2] = toStorageLevelThreeBottom;
    }

    private void configuration()
    {
        ElapsedTime buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        int lockoutTime = 200;
        int index = 0;

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
            else if (gamepad1.right_bumper)
            {
                index = 0;
                endPosition = WAREHOUSE;
                route = WAREHOUSE_ROUTES[index];
            }
            else if (gamepad1.left_bumper)
            {
                index = 0;
                endPosition = STORAGE_UNIT;
                route = STORAGE_ROUTES[index];
            }

            if (endPosition.equals(WAREHOUSE) && gamepad1.square && buttonTime.time() > lockoutTime)
            {
                index = (index + 1) % WAREHOUSE_ROUTES.length;
                route = WAREHOUSE_ROUTES[index];
                buttonTime.reset();
            }
            else if (gamepad1.square && buttonTime.time() > lockoutTime)
            {
                index = (index + 1) % STORAGE_ROUTES.length;
                route = STORAGE_ROUTES[index];
                buttonTime.reset();
            }

            telemetry.addData("Initial Wait Time", initialWaitTime / 1000);
            telemetry.addData("End Position", endPosition);
            telemetry.addData("Route", route);
            telemetry.update();
        }

        telemetry.addData("Status", "Confirmed");
        telemetry.update();
    }
}
