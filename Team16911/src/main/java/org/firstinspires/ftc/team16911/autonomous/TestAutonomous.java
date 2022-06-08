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

@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends LinearOpMode
{
    private SampleMecanumDrive drive;
    private Utilities utilities;


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

        waitForStart();
        if (!opModeIsActive())
        {
            return;
        }

        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(30, 30, 0), Math.toRadians(90))
                .addSpatialMarker(new Vector2d(20, 10),() -> {
                utilities.moveArm(151);

                })
                .splineToSplineHeading(new Pose2d(-20, 10, 0), Math.toRadians(270)).build();

        drive.followTrajectory(traj1);
    }

}
