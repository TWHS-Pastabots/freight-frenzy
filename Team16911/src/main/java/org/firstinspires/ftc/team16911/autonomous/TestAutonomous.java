package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends LinearOpMode
{
    private SampleMecanumDrive drive;

    private int initialWaitTime = 0;

    private final Pose2d positionOne = new Pose2d(8, -5, Math.toRadians(-90));
    private final Pose2d positionTwo = new Pose2d(8, -18, Math.toRadians(-90));
    private final Pose2d positionThree = new Pose2d(0, 27, 0);
    private final Pose2d positionFour = new Pose2d(19, 53, 0);

    public void runOpMode()
    {
        // Configuration Variables
        final int startPosition = 60;

        // Initialize Hardware
        RigatoniHardware hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        Utilities utilities = new Utilities(hardware);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        utilities.moveArm(utilities.positions[1]);

        waitForStart();
        if(!opModeIsActive()) {return;}
        utilities.moveArm(20);


        utilities.intakeCargo();

        Trajectory blockPickupTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(0))
                .splineToSplineHeading(positionOne, Math.toRadians(-90))
                .splineToSplineHeading(positionTwo, Math.toRadians(-90)).build();

        drive.followTrajectory(blockPickupTrajectory);

        utilities.stopIntake();
        utilities.moveArm(utilities.positions[2]);

        Trajectory toHubTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate(), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(0, 0, 0), Math.toRadians(90))
                .splineToLinearHeading(positionThree, Math.toRadians(90))
                .splineToLinearHeading(positionFour, Math.toRadians(30)).build();

        drive.followTrajectory(toHubTrajectory);

        utilities.dropCargo(utilities.CARGO_DROP_TIME, telemetry);
    }
}
