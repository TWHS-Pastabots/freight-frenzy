package org.firstinspires.ftc.team16909.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class FettuccineRR extends LinearOpMode {

    int maxPosition = 100;
    int currentPosition = 0;
    int lastPosition = -100;
    int leftArmOffset = 0;
    //double slowConstant = 1.0;
    boolean justMoved = false;
    boolean canRun = false;
    boolean moveUp = false;
    ElapsedTime armTime = null;
    ElapsedTime armButton = null;
    ElapsedTime timeSinceMove = null;
    FettuccineHardware robot = null;


    double mult = 1.0;
    
    @Override
    public void runOpMode() throws InterruptedException {

        // All Motors
        robot = new FettuccineHardware();
        robot.init(hardwareMap);

        // Drive Motors
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeSinceMove = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();




        while (!isStopRequested()) {

            // CAROUSEL CODE
            if (gamepad2.right_bumper) robot.carousel.setPower(0.6);
            else if (gamepad2.left_bumper) robot.carousel.setPower(-0.6);
            else robot.carousel.setPower(0);

            // ARM PIVOT CODE
            moveArmTwoPointZero();

            // ARM EXTENSION CODE
            if (gamepad2.dpad_left) robot.armServo.setPower(-1.0);
            else if (gamepad2.dpad_right) robot.armServo.setPower(1.0);
            else robot.armServo.setPower(0.0);

            // GRABBER CODE
            if (gamepad2.dpad_up) robot.grabber.setPosition(-1);
            else if (gamepad2.dpad_down) robot.grabber.setPosition(1);

            // MOTOR MULTIPLIERS
            if (gamepad1.circle) mult = 0.25;
            if (gamepad1.cross) mult = 0.5;
            if (gamepad1.square) mult = 0.75;
            if (gamepad1.triangle) mult = 1.0;

            // DRIVE CODE
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * mult,
                            -gamepad1.left_stick_x * mult,
                            -gamepad1.right_stick_x * mult
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();


            /*telemetry.addData("tolerance", robot.rightArm.getTargetPositionTolerance());
            telemetry.addData("rightPower", robot.rightArm.getPower());
            telemetry.addData("leftPower", robot.leftArm.getPower());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();*/
        }
    }

    /*private void moveArm()
    {
        if (gamepad2.right_trigger > 0)
        {
            robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftArm.setPower(gamepad2.right_trigger * 0.7);
            robot.rightArm.setPower(gamepad2.right_trigger * 0.7);
            currentPosition = robot.rightArm.getCurrentPosition();
            leftArmOffset = currentPosition - robot.leftArm.getCurrentPosition();
        }
        else if (gamepad2.left_trigger > 0)
        {
            robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftArm.setPower(gamepad2.left_trigger * -0.01);
            robot.rightArm.setPower(gamepad2.left_trigger * -0.01);
            currentPosition = robot.rightArm.getCurrentPosition();
            leftArmOffset = currentPosition - robot.leftArm.getCurrentPosition();
        }
        else
        {
            robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftArm.setTargetPosition(currentPosition + leftArmOffset);
            robot.rightArm.setTargetPosition(currentPosition);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(1);
            robot.rightArm.setPower(1);
        }

        telemetry.addData("Arm One Pos", robot.rightArm.getCurrentPosition());
        telemetry.addData("Arm One Target", robot.rightArm.getTargetPosition());
        telemetry.addData("Arm Two Pos", robot.leftArm.getCurrentPosition());
        telemetry.addData("Arm Two Target", robot.leftArm.getTargetPosition());
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("LTrigger", gamepad2.left_trigger);
        telemetry.addData("RTrigger", gamepad2.right_trigger);
        telemetry.update();
    }*/


    private void moveArm()
    {
        currentPosition = robot.rightArm.getCurrentPosition();
        leftArmOffset = robot.leftArm.getCurrentPosition() - currentPosition ;

        // Runs driver controlled code
        if (gamepad2.right_trigger > 0)
        {
            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//            robot.rightArm.setPower(getUpwardPower(robot.rightArm.getCurrentPosition()));
//            robot.leftArm.setPower(getUpwardPower(robot.leftArm.getCurrentPosition()));


            // robot.rightArm.setPower(getUpwardPower(robot.rightArm.getCurrentPosition()));
            // robot.leftArm.setPower(getUpwardPower(robot.leftArm.getCurrentPosition()));
            robot.rightArm.setPower(gamepad2.right_trigger * 1);
            robot.leftArm.setPower(gamepad2.right_trigger * 1);


            justMoved = true;
            timeSinceMove.reset();
        }
        else if (gamepad2.left_trigger > 0) {
            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

//            robot.rightArm.setPower(getDownwardPower(robot.rightArm.getCurrentPosition()));
//            robot.leftArm.setPower(getDownwardPower(robot.leftArm.getCurrentPosition()));


            // robot.rightArm.setPower(getDownwardPower(robot.rightArm.getCurrentPosition()));
            // robot.leftArm.setPower(getDownwardPower(robot.leftArm.getCurrentPosition()));
            robot.rightArm.setPower(gamepad2.left_trigger * 0);
            robot.leftArm.setPower(gamepad2.left_trigger * 0);



            justMoved = true;
            timeSinceMove.reset();
        }
        else if (justMoved)
        {
            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition());
            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition());

            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.rightArm.setPower(1);
            robot.leftArm.setPower(1);

            justMoved = false;
        }

        if (currentPosition == lastPosition)
        {
            if (!canRun)
            {
                armTime.reset();
            }
            canRun = true;
        }
        else
        {
            lastPosition = currentPosition;
            canRun = false;
        }

        if (canRun && currentPosition == lastPosition && armTime.milliseconds() >= 100)
        {
            robot.leftArm.setTargetPosition(currentPosition);
            robot.rightArm.setTargetPosition(currentPosition + leftArmOffset);
            // robot.leftArm.setTargetPosition(currentPosition + leftArmOffset);

            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.rightArm.setPower(1);
            robot.leftArm.setPower(1);
        }



        // Runs to highest position
        if (gamepad2.triangle)
        {
            robot.rightArm.setTargetPosition(maxPosition);
            robot.leftArm.setTargetPosition(maxPosition + leftArmOffset);

            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.rightArm.setPower(.7);
            robot.leftArm.setPower(.7);

            justMoved = false;
        }

        telemetry.addData("Arm One Pos", robot.rightArm.getCurrentPosition());
        telemetry.addData("Arm One Target", robot.rightArm.getTargetPosition());
        telemetry.addData("Arm Two Pos", robot.leftArm.getCurrentPosition());
        telemetry.addData("Arm Two Target", robot.leftArm.getTargetPosition());
        telemetry.addData("Current Position", currentPosition);
        telemetry.update();
    }

    private double getUpwardPower(int position)
    {
        return -0.000137 * position * position + position * 0.0137 + 0.189;
    }

    private double getDownwardPower(int position)
    {
        return 0.00006 * position * position - 0.007 * position + 0.1;
    }

    private void moveArmTwoPointZero()
    {
        int rightTargetPos = robot.rightArm.getTargetPosition();
        int leftTargetPos = robot.leftArm.getTargetPosition();

        if (gamepad2.right_trigger > 0)
        {
            robot.rightArm.setTargetPosition(Math.min(rightTargetPos + 1, 100));
            robot.leftArm.setTargetPosition(Math.min(leftTargetPos + 1, 100));

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (gamepad2.left_trigger > 0 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(Math.max(0, rightTargetPos - 1));
            robot.leftArm.setTargetPosition(Math.max(0, leftTargetPos - 1));

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }

        telemetry.addData("Arm One Pos", robot.rightArm.getCurrentPosition());
        telemetry.addData("Arm One Target", robot.rightArm.getTargetPosition());
        telemetry.addData("Arm Two Pos", robot.leftArm.getCurrentPosition());
        telemetry.addData("Arm Two Target", robot.leftArm.getTargetPosition());
        telemetry.addData("Arm Time", armTime.time());
        telemetry.addData("LTrigger", gamepad2.left_trigger);
        telemetry.addData("RTrigger", gamepad2.right_trigger);
        telemetry.update();
    }
}

