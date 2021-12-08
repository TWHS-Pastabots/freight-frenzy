package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16912.util.LinguineHardware;

@Autonomous(name = "Pose Test")
public class PoseTest extends LinearOpMode {

    LinguineHardware robot = new LinguineHardware();
    SampleMecanumDrive drive;

    Trajectory trajectory;

    public void runOpMode() {

        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        initTrajectories();
        waitForStart();

        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        drive.followTrajectory(trajectory);

    }

    private void initTrajectories() {

        trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(5, 0))
                .build();
    }
}
