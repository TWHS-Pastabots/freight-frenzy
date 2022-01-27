package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.team16909.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "Left BLUE")
public class BlueLeft extends LinearOpMode
{
    int current_pos = 0;
    //int leftArmOffset = 0;
    boolean justMoved = true;
    ElapsedTime arm_time = null;
    int target_pos = 0;

    SpaghettiHardware robot = null;
    SampleMecanumDrive drive = null;

    //Positions
    private final Pose2d scan_pos = new Pose2d(69, 69, 69);
    private final Pose2d hub_pos = new Pose2d(69, 69, 69);
    private final Pose2d aim_pos = new Pose2d(69, 69, 69);
    private final Pose2d warehouse_pos = new Pose2d(69, 69, 69);

    //Trajectories
    private TrajectorySequence scan_traj;
    private TrajectorySequence hub_traj;
    private TrajectorySequence aim_traj;
    private TrajectorySequence warehouse_traj;

    @Override
    public void runOpMode() throws InterruptedException
    {

        // All Motors
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Action Method Init
        //action = new helpMethods(robot);

        blueLeftTraj();

        while (!isStarted()) {
            if (gamepad2.dpad_right) target_pos = 0;
            else if (gamepad2.dpad_down) target_pos = 1;
            else if (gamepad2.dpad_left) target_pos = 2;
            else if (gamepad2.dpad_up) target_pos = 3;

            telemetry.addData("Arm Target", target_pos);
            telemetry.update();
        }

        waitForStart();
        if (!opModeIsActive()) return;
        robot.grabberServo.setPosition(0.2);
        drive.followTrajectorySequence(scan_traj);
        helpMethods.moveArmv2(robot, target_pos);
        helpMethods.waitFor(0.2);
        drive.followTrajectorySequence(hub_traj);
        robot.grabberServo.setPosition(0.2);
        helpMethods.waitFor(0.2);
        drive.followTrajectorySequence(aim_traj);
        drive.followTrajectorySequence(warehouse_traj);
        helpMethods.moveArmv2(robot,2);

    }

    private void blueLeftTraj()
    {
        scan_traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(scan_pos)
                .build();
        hub_traj = drive.trajectorySequenceBuilder(scan_traj.end())
                .lineToLinearHeading(hub_pos)
                .forward(3)
                .build();
        aim_traj = drive.trajectorySequenceBuilder(hub_traj.end())
                .back(2)
                .lineToLinearHeading(aim_pos)
                .back(1)
                .build();
        warehouse_traj = drive.trajectorySequenceBuilder(aim_traj.end())
                .lineToLinearHeading(warehouse_pos)
                .forward(69)
                .build();
    }




}
