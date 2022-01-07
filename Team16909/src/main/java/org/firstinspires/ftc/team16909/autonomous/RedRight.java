package org.firstinspires.ftc.team16909.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;
import org.firstinspires.ftc.team16909.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "FettuccineRRv2")
public class RedRight extends LinearOpMode {

    int currentPosition = 0;
    int leftArmOffset = 0;
    boolean justMoved = false;
    ElapsedTime armTime = null;

    int tgtPos = 0;

    FettuccineHardware robot = null;
    SampleMecanumDrive drive = null;

    String autonState = "redLeft";

    double mult = 0.5;

    // Poses
    private final Pose2d posScan = new Pose2d(15.2, -5.0401, Math.toRadians(0));
    private final Pose2d posHub = new Pose2d(19.3338, 21.4224, Math.toRadians(0));
    private final Pose2d posApproach = new Pose2d(-1, 0, 0);
    private final Pose2d posWarehouse = new Pose2d(-5, -36.3931, Math.toRadians(0));

    // Trajectories
    private TrajectorySequence trajScan, trajHub, trajWarehouse, trajApproach;

    ActionMethods action;

    @Override
    public void runOpMode() throws InterruptedException {

        // All Motors
        robot = new FettuccineHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Action Method Init
        action = new ActionMethods(robot);

        blueLeftTrajectories();

        while (!isStarted()) {
            if (gamepad2.dpad_right) tgtPos = 0;
            else if (gamepad2.dpad_down) tgtPos = 1;
            else if (gamepad2.dpad_left) tgtPos = 2;
            else if (gamepad2.dpad_up) tgtPos = 3;

            telemetry.addData("Arm Target", tgtPos);
            telemetry.update();
        }


        waitForStart();
        if (!opModeIsActive()) return;
        robot.grabber.setPosition(0.2);
        drive.followTrajectorySequence(trajScan);
        action.moveArm(tgtPos);
        action.waitFor(1);
        drive.followTrajectorySequence(trajHub);
        robot.grabber.setPosition(0.5);
        action.waitFor(1);
        drive.followTrajectorySequence(trajApproach);
        drive.followTrajectorySequence(trajWarehouse);
        action.moveArm(0);

    }


    // Trajectory Build

    private void blueLeftTrajectories ()
    {
        trajScan = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(posScan)
                .build();
        trajHub = drive.trajectorySequenceBuilder(trajScan.end())
                .lineToLinearHeading(posHub)
                .forward(4)
                .build();
        trajApproach = drive.trajectorySequenceBuilder(trajHub.end())
                .back(4)
                .lineToLinearHeading(posApproach)
                .back(4)
                .build();
        trajWarehouse = drive.trajectorySequenceBuilder(trajApproach.end())
                .lineToLinearHeading(posWarehouse)
                .forward(12)
                .build();
    }


    // Action Methods


    private void moveCarousel (double time, double power) {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {
            robot.carousel.setPower(power);
        }
        robot.carousel.setPower(0);
    }

    /*
    private void moveX (double time, double power) {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0 * mult,
                            -power * mult,
                            0 * mult
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
        }
    }

    private void rotate (double time, double power) {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0 * mult,
                            0 * mult,
                            -power * mult
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
        }
    }
    */

    // ARM AUTON

    private void moveArmTwoPointZero(int armUp)
    {
        int rightTargetPos = robot.rightArm.getTargetPosition();
        int leftTargetPos = robot.leftArm.getTargetPosition();
        int offset = Math.abs(robot.rightArm.getCurrentPosition() - robot.rightArm.getTargetPosition());
        currentPosition = robot.rightArm.getCurrentPosition();
        leftArmOffset = currentPosition - robot.leftArm.getCurrentPosition();

        if (armUp == 1 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(170);
            robot.leftArm.setTargetPosition(170);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() + 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() + 10);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (gamepad2.circle && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(0);
            robot.leftArm.setTargetPosition(0);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() - 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() - 10);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (offset >= 5 && armTime.time() >= 500)
        {
            robot.leftArm.setTargetPosition(currentPosition + leftArmOffset);
            robot.rightArm.setTargetPosition(currentPosition);
            // robot.leftArm.setTargetPosition(currentPosition + leftArmOffset);

            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.rightArm.setPower(1);
            robot.leftArm.setPower(1);

            justMoved = false;
        }

        /*if (currentPosition == lastPosition)
        {
            if (!canRun)
            {
                lockoutTime.reset();
            }
            canRun = true;
        }
        else
        {
            lastPosition = currentPosition;
            canRun = false;
        }*/

        telemetry.addData("rF", robot.rightFront.getVelocity());
        telemetry.addData("lF", robot.leftFront.getVelocity());
        telemetry.addData("rR", robot.rightRear.getVelocity());
        telemetry.addData("lR", robot.leftRear.getVelocity());
        telemetry.addData("Arm One Pos", robot.rightArm.getCurrentPosition());
        telemetry.addData("Arm One Target", robot.rightArm.getTargetPosition());
        telemetry.addData("Arm Two Pos", robot.leftArm.getCurrentPosition());
        telemetry.addData("Arm Two Target", robot.leftArm.getTargetPosition());
        telemetry.addData("Arm Time", armTime.time());
        telemetry.addData("Just Moved", justMoved);
        telemetry.update();
    }


}
