package org.firstinspires.ftc.team16911.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16911.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16911.hardware.TestHardware;



public class TestAutonomous extends LinearOpMode
{
    private TestHardware hardware;

    public void runOpMode()
    {
        // Initialization
        hardware.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        Trajectory firstTrajectory = drive.trajectoryBuilder(new Pose2d()).forward(15).build();

        waitForStart();
        if(!opModeIsActive()) {return;}

        drive.followTrajectory(firstTrajectory);
    }
}
