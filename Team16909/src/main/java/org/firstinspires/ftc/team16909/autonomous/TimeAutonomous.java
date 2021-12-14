package org.firstinspires.ftc.team16909.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;

@Autonomous(preselectTeleOp = "FettuccineRRv2")
public class TimeAutonomous extends LinearOpMode {

    int maxPosition = 100;
    int currentPosition = 0;
    int lastPosition = -100;
    int leftArmOffset = 0;
    //double slowConstant = 1.0;
    boolean grabHold = false;
    boolean justMoved = false;
    boolean canRun = false;
    boolean moveUp = false;
    ElapsedTime armTime = null;
    ElapsedTime lockoutTime = null;
    ElapsedTime armButton = null;
    ElapsedTime timeSinceMove = null;

    FettuccineHardware robot = null;
    SampleMecanumDrive drive = null;

    String autonState = null;

    double mult = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // All Motors
        robot = new FettuccineHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStarted()) {
            if (gamepad2.circle) autonState = "redLeft";
            else if (gamepad2.cross) autonState = "redRight";
            else if (gamepad2.square) autonState = "blueLeft";
            else if (gamepad2.triangle) autonState = "blueRight";

            telemetry.addData("Auton", autonState);
            telemetry.update();
        }

        waitForStart();

        switch (autonState) {
            case "redLeft":
                redLeft();
                break;
            case "redRight":
                redRight();
                break;
            case "blueLeft":
                blueLeft();
                break;
            case "blueRight":
                blueRight();
                break;
        }

    }


    // AUTON METHODS

    private void redLeft () {

        moveX(3, 0.75);
        moveY(0.5, -0.5);
        moveX(3, 0.75);
    }

    private void redRight () {
        moveX(2, 0.75);
        moveY(0.5, -0.5);
        moveX(1.5, 0.75);
    }

    private void blueLeft () {
        moveX(2, -0.75);
        moveY(0.5, -0.5);
        moveX(1.5, -0.75);
    }

    private void blueRight () {
        moveX(3, -0.75);
        moveY(0.5, -0.5);
        moveX(3, -0.75);
    }


    // MOTION METHODS


    private void moveY (double time, double power) {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            power * mult,
                            0 * mult,
                            0 * mult
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
        }
    }

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
