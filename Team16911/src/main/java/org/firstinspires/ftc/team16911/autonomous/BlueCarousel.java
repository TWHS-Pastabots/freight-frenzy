package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@Autonomous(name = "BlueCarousel")
public class BlueCarousel extends LinearOpMode
{
    private SampleMecanumDrive drive;

    private static final String WAREHOUSE = "Warehouse";
    private static final String STORAGE_UNIT = "Storage Unit";
    private static final String DIRECT_ROUTE = "Direct";
    private static final String BOTTOM_ROUTE = "Bottom";
    private static final String WAREHOUSE_DIRECT_ROUTE = WAREHOUSE + DIRECT_ROUTE;
    private static final String WAREHOUSE_BOTTOM_ROUTE = WAREHOUSE + BOTTOM_ROUTE;
    private static final String STORAGE_DIRECT_ROUTE = STORAGE_UNIT + DIRECT_ROUTE;
    private static final String[] WAREHOUSE_ROUTES = {DIRECT_ROUTE, BOTTOM_ROUTE};
    private static final String[] STORAGE_ROUTES = {DIRECT_ROUTE};
    private static String endPosition = WAREHOUSE;
    private static String route = DIRECT_ROUTE;

    private int initialWaitTime = 0;

    private final Pose2d carousel = new Pose2d(5, -21.5, 0);
    private final Pose2d barcode = new Pose2d(18.5,0, 0);
    private final Pose2d hubLevelOne = new Pose2d(17.3, 27.75, 0);
    private final Pose2d hubLevelTwo = new Pose2d(17.7, 27.75, 0);
    private final Pose2d hubLevelThree = new Pose2d(24, 27.75, 0);
    private final Pose2d warehouseOutside = new Pose2d(-.25, 60, 0);
    private final Pose2d warehouseBottomPosition = new Pose2d(3, 36, 0);
    private final Pose2d warehouse = new Pose2d(-.25, 81, 0);
    private final Pose2d storageUnit = new Pose2d(27,-21, Math.toRadians(90));
    private final Pose2d storageSide = new Pose2d(43, -11.5, Math.toRadians(90));
    private final Pose2d storageHubLevelOne = new Pose2d(42.2, 5.9, Math.toRadians(90));
    private final Pose2d storageHubLevelTwo = new Pose2d(42.2, 6.8, Math.toRadians(90));
    private final Pose2d storageHubLevelThree = new Pose2d(42.2, 13, Math.toRadians(90));

    private Trajectory toCarousel, toBarcode;

    private final Trajectory[] toHubTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubDirectTrajectories = new Trajectory[3];
    private final Trajectory[] fromHubBottomTrajectories = new Trajectory[3];
    private final Trajectory[] toStorageHubTrajectories = new Trajectory[3];
    private final Trajectory[] toStorageTrajectories = new Trajectory[3];

    public void runOpMode()
    {
        // Initialize Hardware
        RigatoniHardware hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        Utilities utilities = new Utilities(hardware);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        buildTrajectories();

        configuration();
        final String path = endPosition + route;

        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.wait(initialWaitTime, telemetry);
        utilities.moveArm(utilities.initialArmPosition);

        drive.followTrajectory(toCarousel);
        hardware.carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        utilities.spinCarousel(2700, telemetry);

        drive.followTrajectory(toBarcode);
        int barcodeLevel = utilities.getBarcodeLevelBlueSide();
        if (barcodeLevel != 0) { utilities.moveArm(utilities.positions[barcodeLevel]); }
        telemetry.addData("Right Distance", hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left Distance", hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        switch (path)
        {
            case WAREHOUSE_DIRECT_ROUTE:
                drive.followTrajectory(toHubTrajectories[barcodeLevel]);
                if (barcodeLevel == 0)
                {
                    utilities.moveArm(utilities.positions[barcodeLevel]);
                    utilities.wait(750, telemetry);
                }
                utilities.eliminateOscillations();
                utilities.dropCargo(utilities.CARGO_DROP_TIME, utilities.DROP_POWERS[barcodeLevel],telemetry);
                drive.followTrajectory(fromHubDirectTrajectories[barcodeLevel]);
                break;
            case WAREHOUSE_BOTTOM_ROUTE:
                drive.followTrajectory(toHubTrajectories[barcodeLevel]);
                if (barcodeLevel == 0)
                {
                    utilities.moveArm(utilities.positions[barcodeLevel]);
                    utilities.wait(750, telemetry);
                }
                utilities.eliminateOscillations();
                utilities.dropCargo(utilities.CARGO_DROP_TIME, utilities.DROP_POWERS[barcodeLevel],telemetry);
                drive.followTrajectory(fromHubBottomTrajectories[barcodeLevel]);
                break;
            case STORAGE_DIRECT_ROUTE:
                drive.followTrajectory(toStorageHubTrajectories[barcodeLevel]);
                if (barcodeLevel == 0)
                {
                    utilities.moveArm(utilities.positions[barcodeLevel]);
                    utilities.wait(750, telemetry);
                }
                utilities.eliminateOscillations();
                utilities.dropCargo(utilities.CARGO_DROP_TIME, utilities.DROP_POWERS[barcodeLevel],telemetry);
                drive.followTrajectory(toStorageTrajectories[barcodeLevel]);
        }
    }

    private void buildTrajectories()
    {
        // Configuration Variables
        Trajectory toHubLevelOne, toHubLevelTwo, toHubLevelThree;
        Trajectory fromHubLevelOneDirect, fromHubLevelTwoDirect, fromHubLevelThreeDirect;
        Trajectory fromHubLevelOneBottom, fromHubLevelTwoBottom, fromHubLevelThreeBottom;
        Trajectory toHubOneStorageSide, toHubTwoStorageSide, toHubThreeStorageSide;
        Trajectory toStorageLevelOne, toStorageLevelTwo, toStorageLevelThree;

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

        fromHubLevelOneDirect = drive.trajectoryBuilder(toHubLevelOne.end(), Math.toRadians(130))
                .splineToLinearHeading(warehouseOutside, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        fromHubLevelTwoDirect = drive.trajectoryBuilder(toHubLevelTwo.end(), Math.toRadians(130))
                .splineToLinearHeading(warehouseOutside, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        fromHubLevelThreeDirect = drive.trajectoryBuilder(toHubLevelThree.end(), Math.toRadians(130))
                .splineToLinearHeading(warehouseOutside, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        fromHubLevelOneBottom = drive.trajectoryBuilder(toHubLevelOne.end(), Math.toRadians(180))
                .splineToLinearHeading(warehouseBottomPosition, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        fromHubLevelTwoBottom = drive.trajectoryBuilder(toHubLevelTwo.end(), Math.toRadians(180))
                .splineToLinearHeading(warehouseBottomPosition, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        fromHubLevelThreeBottom = drive.trajectoryBuilder(toHubLevelThree.end(), Math.toRadians(180))
                .splineToLinearHeading(warehouseBottomPosition, Math.toRadians(90))
                .splineToLinearHeading(warehouse, Math.toRadians(90)).build();

        toHubOneStorageSide = drive.trajectoryBuilder(toBarcode.end(), Math.toRadians(270))
                .splineToSplineHeading(storageUnit, Math.toRadians(0))
                .splineToLinearHeading(storageSide, Math.toRadians(90))
                .splineToLinearHeading(storageHubLevelOne, Math.toRadians(90)).build();

        toHubTwoStorageSide = drive.trajectoryBuilder(toBarcode.end(), Math.toRadians(270))
                .splineToSplineHeading(storageUnit, Math.toRadians(0))
                .splineToLinearHeading(storageSide, Math.toRadians(90))
                .splineToLinearHeading(storageHubLevelTwo, Math.toRadians(90)).build();

        toHubThreeStorageSide = drive.trajectoryBuilder(toBarcode.end(), Math.toRadians(270))
                .splineToSplineHeading(storageUnit, Math.toRadians(0))
                .splineToLinearHeading(storageSide, Math.toRadians(90))
                .splineToLinearHeading(storageHubLevelThree, Math.toRadians(90)).build();

        toStorageLevelOne = drive.trajectoryBuilder(toHubOneStorageSide.end(), Math.toRadians(270))
                .splineToSplineHeading(storageSide, Math.toRadians(270))
                .splineToLinearHeading(storageUnit, Math.toRadians(180)).build();

        toStorageLevelTwo = drive.trajectoryBuilder(toHubTwoStorageSide.end(), Math.toRadians(270))
                .splineToSplineHeading(storageSide, Math.toRadians(270))
                .splineToLinearHeading(storageUnit, Math.toRadians(180)).build();

        toStorageLevelThree = drive.trajectoryBuilder(toHubThreeStorageSide.end(), Math.toRadians(270))
                .splineToSplineHeading(storageSide, Math.toRadians(270))
                .splineToLinearHeading(storageUnit, Math.toRadians(180)).build();

        toHubTrajectories[0] = toHubLevelOne;
        toHubTrajectories[1] = toHubLevelTwo;
        toHubTrajectories[2] = toHubLevelThree;

        fromHubDirectTrajectories[0] = fromHubLevelOneDirect;
        fromHubDirectTrajectories[1] = fromHubLevelTwoDirect;
        fromHubDirectTrajectories[2] = fromHubLevelThreeDirect;

        fromHubBottomTrajectories[0] = fromHubLevelOneBottom;
        fromHubBottomTrajectories[1] = fromHubLevelTwoBottom;
        fromHubBottomTrajectories[2] = fromHubLevelThreeBottom;

        toStorageHubTrajectories[0] = toHubOneStorageSide;
        toStorageHubTrajectories[1] = toHubTwoStorageSide;
        toStorageHubTrajectories[2] = toHubThreeStorageSide;

        toStorageTrajectories[0] = toStorageLevelOne;
        toStorageTrajectories[1] = toStorageLevelTwo;
        toStorageTrajectories[2] = toStorageLevelThree;
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
                //index = (index + 1) % STORAGE_ROUTES.length;
                index = 0;
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
