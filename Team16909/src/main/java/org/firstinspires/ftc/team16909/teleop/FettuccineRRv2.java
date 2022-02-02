package org.firstinspires.ftc.team16909.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class FettuccineRRv2 extends LinearOpMode {

    // Arm Motion/Grabber Variables
    boolean grabHold = false;
    boolean justMoved = false;
    ElapsedTime armTime = null;
    ElapsedTime lockoutTime = null;
    ElapsedTime timeSinceMove = null;

    // Hardware Instantiation
    FettuccineHardware robot = null;
    SampleMecanumDrive drive = null;

    // Arm Values
    int currentPosition = 0;
    int leftArmOffset = 0;

    // Dpad Strafe Values
    int dpadFB = 0;
    int dpadLR = 0;

    // Motor Multiplier Values
    double mult = 0.5;
    
    @Override
    public void runOpMode() throws InterruptedException
    {

        // All Motors
        robot = new FettuccineHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Arm Variables
        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeSinceMove = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        lockoutTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // LOOP BELOW

        waitForStart();
        while (!isStopRequested()) {

            // External Op Methods
            //armExtensionCode();
            armPivotCode();
            //grabberPivotCode();
            carouselCode();
            grabberCode();

            // Dpad Strafing Code
            if (gamepad1.dpad_up) dpadFB = 1;
            else if (gamepad1.dpad_down) dpadFB = -1;
            else dpadFB = 0;
            if (gamepad1.dpad_left) dpadLR = 1;
            else if (gamepad1.dpad_right) dpadLR = -1;
            else dpadLR = 0;

            // Drive Multiplier Code
            if (gamepad1.circle) mult = 0.25;
            if (gamepad1.cross) mult = 0.5;
            if (gamepad1.square) mult = 0.75;
            if (gamepad1.triangle) mult = 1.0;

            // Drive Main Code
            drive.setWeightedDrivePower(
                    new Pose2d(
                            (gamepad1.left_stick_x + dpadLR) * mult,
                            (-gamepad1.left_stick_y + dpadFB) * mult,
                            -gamepad1.right_stick_x * mult
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }

   /* private void grabberPivotCode()
    {
        robot.grabberPivot.setPower(gamepad2.left_stick_y * 0.1);
    }
*/

    private void armPivotCode()
    {
        int rightTargetPos = robot.rightArm.getTargetPosition();
        int leftTargetPos = robot.leftArm.getTargetPosition();
        int offset = Math.abs(robot.rightArm.getCurrentPosition() - robot.rightArm.getTargetPosition());
        currentPosition = robot.rightArm.getCurrentPosition();
        leftArmOffset = currentPosition - robot.leftArm.getCurrentPosition();

        if (gamepad2.right_trigger > 0 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(rightTargetPos + 20);
            robot.leftArm.setTargetPosition(leftTargetPos + 20);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (gamepad2.left_trigger > 0 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(Math.max(-40, rightTargetPos - 20));
            robot.leftArm.setTargetPosition(Math.max(-40, leftTargetPos - 20));

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (gamepad2.triangle && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(270);
            robot.leftArm.setTargetPosition(270);
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
            robot.rightArm.setTargetPosition(-40);
            robot.leftArm.setTargetPosition(-40);
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

//        telemetry.addData("rF", robot.rightFront.getVelocity());
//        telemetry.addData("lF", robot.leftFront.getVelocity());
//        telemetry.addData("rR", robot.rightRear.getVelocity());
//        telemetry.addData("lR", robot.leftRear.getVelocity());
//        telemetry.addData("Arm One Pos", robot.rightArm.getCurrentPosition());
//        telemetry.addData("Arm One Target", robot.rightArm.getTargetPosition());
//        telemetry.addData("Arm Two Pos", robot.leftArm.getCurrentPosition());
//        telemetry.addData("Arm Two Target", robot.leftArm.getTargetPosition());
//        telemetry.addData("Arm Time", armTime.time());
//        telemetry.addData("Just Moved", justMoved);
//        telemetry.update();
    }

    private void carouselCode()
    {
        if (gamepad2.right_bumper) robot.carousel.setPower(0.6);
        else if (gamepad2.left_bumper) robot.carousel.setPower(-0.6);
        else robot.carousel.setPower(0);
    }

    /*private void armExtensionCode()
    {
        if (gamepad2.dpad_left) robot.armServo.setPower(-1.0);
        else if (gamepad2.dpad_right) robot.armServo.setPower(1.0);
        else robot.armServo.setPower(0.0);
    }*/

    private void grabberCode()
    {
        if (gamepad2.dpad_up) grabHold = true;
        else if (gamepad2.dpad_down) grabHold = false;

        if (grabHold) robot.grabber.setPosition(0);
        else if (!grabHold) robot.grabber.setPosition(0.5);
    }
}

