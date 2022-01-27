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

    private final Pose2d positionOne = new Pose2d(15, 0, 0);
    private final Pose2d positionTwo = new Pose2d(30, -15, 0);
    private final Pose2d positionThree = new Pose2d(30, 15, 0);
    private final Pose2d positionFour = new Pose2d(30,  0, 0);
    private final Pose2d positionFive = new Pose2d(0, 0, 0);


    public void runOpMode()
    {
        // Configuration Variables
        final int startPosition = 60;

        // Initialize Hardware
        RigatoniHardware hardware = new RigatoniHardware();
        hardware.init(hardwareMap);

        // Initialize Mecanum Drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());


        waitForStart();
        if(!opModeIsActive()) {return;}

        while (!gamepad1.circle)
        {
            continue;
        }

        Trajectory firstTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(positionOne).build();

        drive.followTrajectory(firstTrajectory);

        while (!gamepad1.circle)
        {
            continue;
        }

        Trajectory secondTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(positionTwo).build();

        drive.followTrajectory(secondTrajectory);

        while (!gamepad1.circle)
        {
            continue;
        }

        Trajectory thirdTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(positionThree).build();

        drive.followTrajectory(thirdTrajectory);

        while (!gamepad1.circle)
        {
            continue;
        }

        Trajectory fourthTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(positionFour).build();

        drive.followTrajectory(fourthTrajectory);

        while (!gamepad1.circle)
        {
            continue;
        }

        Trajectory fifthTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(positionFive).build();

        drive.followTrajectory(fifthTrajectory);
    }

}
