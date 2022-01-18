package org.firstinspires.ftc.team16909.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class RedLeft extends LinearOpMode {

    int currentPosition = 0;
    int leftArmOffset = 0;
    boolean justMoved = false;
    ElapsedTime armTime = null;

    FettuccineHardware robot = null;
    SampleMecanumDrive drive = null;

    String autonState = "redLeft";

    double mult = 0.5;

    // Poses
    private final Pose2d posPushback = new Pose2d(4.7063, 0,Math.toRadians(0));
    private final Pose2d posBlueCarousel = new Pose2d(6.9063, 17.9322, Math.toRadians(90));
    private final Pose2d posBlueUnitPark = new Pose2d(32, 22.5, Math.toRadians(-90));

    // Trajectories
    private Trajectory trajPushback, trajBlueCarousel, trajBlueUnitPark;

    @Override
    public void runOpMode() throws InterruptedException {

        // All Motors
        robot = new FettuccineHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Action Method Init
        ActionMethods action = new ActionMethods(robot);

        blueRightTrajectories();

        waitForStart();
        if (!opModeIsActive()) return;

        drive.followTrajectory(trajPushback);
        drive.followTrajectory(trajBlueCarousel);
        action.moveCarousel(4,0.5);
        drive.followTrajectory(trajBlueUnitPark);


    }


    // Trajectory Build

    private void blueRightTrajectories ()
    {
        trajPushback = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(posPushback)
                .build();
        trajBlueCarousel = drive.trajectoryBuilder(trajPushback.end())
                .lineToSplineHeading(posBlueCarousel)
                .build();
        trajBlueUnitPark = drive.trajectoryBuilder(trajBlueCarousel.end())
                .lineToSplineHeading(posBlueUnitPark)
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
