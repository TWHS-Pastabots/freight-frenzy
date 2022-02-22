package org.firstinspires.ftc.team16910.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;

@TeleOp(name = "Spaghetti")
public class Spaghetti extends OpMode {

    SpaghettiHardware robot;
    int maxPosition = 100;
    int currentPosition = 0;
    int lastPosition = -100;
    int armMotorTwoOffset = 0;
    ElapsedTime armTime = null;
    boolean canRun = false;
    boolean justMoved = false;
    double tempSpeedMultiplier = 1.5;
    ElapsedTime elapsedTime = new ElapsedTime();

    public void init()
    {
        // Initialize robot
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);
        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop()
    {
        drive();
        moveArm();
        spinCarousel();
        servos();
    }

    public void stop()
    {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private void drive()
    {
        if(!gamepad1.right_bumper) {
            // Mecanum drivecode
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double special_y = -gamepad1.left_stick_y * .8;
            double rx = gamepad1.left_stick_x;// Counteract imperfect strafing
            double special_x = -gamepad1.left_stick_x * .8;
            double x = -gamepad1.right_stick_x;
            double special_rx = -gamepad1.right_trigger * .6;

            double rightFrontPower = y + x + (rx * 0.6);
            double rightRearPower = y - x + (rx * 0.6);
            double leftFrontPower = y - x - (rx * 0.6);
            double leftRearPower = y + x - (rx * 0.6);

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 ||
                    Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {
                // Find the largest power
                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                max = Math.max(Math.abs(rightFrontPower), max);
                max = Math.max(Math.abs(rightRearPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            //Slowmode Code
            if (gamepad1.dpad_up) {
                leftFrontPower = -.2;
                leftRearPower = -.2;
                rightRearPower = -.2;
                rightFrontPower = -.2;

            } else if (gamepad1.dpad_down) {
                leftFrontPower = .2;
                leftRearPower = .2;
                rightRearPower = .2;
                rightFrontPower = .2;
            } else if (gamepad1.dpad_right) {
                leftFrontPower = -.2;
                leftRearPower = -.2;
                rightRearPower = .2;
                rightFrontPower = .2;
            } else if (gamepad1.dpad_left) {
                leftFrontPower = .2;
                leftRearPower = .2;
                rightRearPower = -.2;
                rightFrontPower = -.2;
            }

            if (gamepad1.right_trigger > 0) {
                robot.leftFront.setPower(.5);
                robot.leftRear.setPower(.5);
                robot.rightRear.setPower(-.5);
                robot.rightFront.setPower(-.5);
            }

            if (gamepad1.left_trigger > 0) {
                robot.leftFront.setPower(-.5);
                robot.leftRear.setPower(-.5);
                robot.rightRear.setPower(.5);
                robot.rightFront.setPower(.5);
            }

            robot.leftFront.setPower(leftFrontPower * tempSpeedMultiplier);
            robot.leftRear.setPower(leftRearPower * tempSpeedMultiplier);
            robot.rightFront.setPower(rightFrontPower * tempSpeedMultiplier);
            robot.rightRear.setPower(rightRearPower * tempSpeedMultiplier);

        }
        if(gamepad1.right_bumper)
        {
            // Mecanum drivecode
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double special_y = gamepad1.left_stick_y * .8;
            double rx = -gamepad1.left_stick_x;// Counteract imperfect strafing
            double special_x = gamepad1.left_stick_x * .8;
            double x = -gamepad1.right_stick_x;
            double special_rx = gamepad1.right_trigger * .6;

            double rightFrontPower = (y*.2) + (x*.2) + (rx * 0.2);
            double rightRearPower = (y*.2) - (x*.2) + (rx * 0.2);
            double leftFrontPower = (y*.2) - (x*.2) - (rx * 0.2);
            double leftRearPower = (y*.2) + (x*.2) - (rx * 0.2);

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 ||
                    Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {
                // Find the largest power
                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                max = Math.max(Math.abs(rightFrontPower), max);
                max = Math.max(Math.abs(rightRearPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            /*//Slowmode Code
            if (gamepad1.dpad_down) {
                leftFrontPower = .2;
                leftRearPower = .2;
                rightRearPower = .2;
                rightFrontPower = .2;

            } else if (gamepad1.dpad_up) {
                leftFrontPower = -.2;
                leftRearPower = -.2;
                rightRearPower = -.2;
                rightFrontPower = -.2;
            } else if (gamepad1.dpad_right) {
                leftFrontPower = -.2;
                leftRearPower = .2;
                rightRearPower = -.2;
                rightFrontPower = .2;
            } else if (gamepad1.dpad_left) {
                leftFrontPower = .2;
                leftRearPower = -.2;
                rightRearPower = .2;
                rightFrontPower = -.2;
            }

            if (gamepad1.right_trigger > 0) {
                robot.leftFront.setPower(-.5);
                robot.leftRear.setPower(-.5);
                robot.rightRear.setPower(.5);
                robot.rightFront.setPower(.5);
            }

            if (gamepad1.left_trigger > 0) {
                robot.leftFront.setPower(.5);
                robot.leftRear.setPower(.5);
                robot.rightRear.setPower(-.5);
                robot.rightFront.setPower(-.5);
            }*/

            robot.leftFront.setPower(leftFrontPower * tempSpeedMultiplier);
            robot.leftRear.setPower(leftRearPower * tempSpeedMultiplier);
            robot.rightFront.setPower(rightFrontPower * tempSpeedMultiplier);
            robot.rightRear.setPower(rightRearPower * tempSpeedMultiplier);
        }


    }

    private void moveArm()
    {
        currentPosition = robot.armMotorOne.getCurrentPosition();
        armMotorTwoOffset = robot.armMotorTwo.getCurrentPosition() - currentPosition;

        // Runs driver controlled code
        if (gamepad2.right_trigger > 0)
        {
            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            robot.armMotorOne.setPower(gamepad2.right_trigger * -.6);
            robot.armMotorTwo.setPower(gamepad2.right_trigger * -.6);

            justMoved = true;
        }
        else if (gamepad2.left_trigger > 0)
        {
            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            robot.armMotorOne.setPower(gamepad2.left_trigger * .6);
            robot.armMotorTwo.setPower(gamepad2.left_trigger * .6);

            justMoved = true;
        }
        else if (justMoved)
        {
            robot.armMotorOne.setTargetPosition(currentPosition);
            robot.armMotorTwo.setTargetPosition(currentPosition + armMotorTwoOffset);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.armMotorOne.setPower(1);
            robot.armMotorTwo.setPower(1);

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

        if (canRun && currentPosition == lastPosition && armTime.milliseconds() >= 150)
        {
            robot.armMotorOne.setTargetPosition(currentPosition);
            robot.armMotorTwo.setTargetPosition(currentPosition + armMotorTwoOffset);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.armMotorOne.setPower(1);
            robot.armMotorTwo.setPower(1);
        }

        // Resets zero position for calibration
        if (gamepad2.dpad_down)
        {
            robot.armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Arm One Pos", robot.armMotorOne.getCurrentPosition());
        telemetry.addData("Arm One Target", robot.armMotorOne.getTargetPosition());
        telemetry.addData("Arm Two Pos", robot.armMotorTwo.getCurrentPosition());
        telemetry.addData("Arm Two Target", robot.armMotorTwo.getTargetPosition());
        telemetry.addData("Current Position", currentPosition);
        telemetry.update();
    }

    private void spinCarousel()

    {
        // Carousel Motor Code
        if (gamepad2.right_bumper)
        {
            //robot.spinnyWheel.setDirection(DcMotorEx.Direction.FORWARD);
            robot.spinnyWheel.setPower(.6);
            //robot.leftSpinnyWheel.setPower(0.6);
        }

        else if (gamepad2.left_bumper)
        {
            robot.spinnyWheel.setDirection(DcMotorEx.Direction.REVERSE);
            robot.spinnyWheel.setPower(-.6);
        }

        else
        {
            robot.spinnyWheel.setPower(0.0);
            //robot.leftSpinnyWheel.setPower(0.0);
        }
    }

    private void servos()
    {

        if(gamepad2.square)
        {
           // robot.doorServo.setPosition(-1);
            robot.grabberServo.setPosition(1);
        }

        if(gamepad2.triangle)
        {
           // robot.doorServo.setPosition(1);
            robot.grabberServo.setPosition(-1);
        }
    }
}
