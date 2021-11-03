package org.firstinspires.ftc.team16910.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    }

    public void stop()
    {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private void drive()
    {
        // Mecanum drivecode
        double y = gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double leftFrontPower = y - x - rx;
        double leftRearPower = y + x - rx;
        double rightFrontPower = y + x + rx;
        double rightRearPower = y - x + rx;

        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 ||
                Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1 )
        {
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

        if (gamepad1.dpad_up || gamepad1.dpad_right)
        {
            leftFrontPower = -1;
            rightRearPower = -1;
            rightFrontPower = 1;
            leftRearPower = 1;
        }
        else if (gamepad1.dpad_down || gamepad1.dpad_left)
        {
            leftFrontPower = 1;
            rightRearPower = 1;
            rightFrontPower = -1;
            leftRearPower = -1;
        }

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
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

            robot.armMotorOne.setPower(getUpwardPower(currentPosition));
            robot.armMotorTwo.setPower(getUpwardPower(currentPosition));

            justMoved = true;
        }
        else if (gamepad2.left_trigger > 0)
        {
            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            robot.armMotorOne.setPower(getDownwardPower(currentPosition));
            robot.armMotorTwo.setPower(getDownwardPower(currentPosition));

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
            robot.armMotorTwo.setTargetPosition(currentPosition);
            robot.armMotorTwo.setTargetPosition(currentPosition + armMotorTwoOffset);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.armMotorOne.setPower(1);
            robot.armMotorTwo.setPower(1);
        }



        // Runs to highest position
        if (gamepad2.triangle)
        {
            robot.armMotorOne.setTargetPosition(maxPosition);
            robot.armMotorTwo.setTargetPosition(maxPosition + armMotorTwoOffset);

            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.armMotorOne.setPower(.7);
            robot.armMotorTwo.setPower(.7);

            justMoved = false;
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
            robot.spinnyWheel.setDirection(DcMotorEx.Direction.FORWARD);
            robot.spinnyWheel.setPower(.6);
        }
        else if (gamepad2.left_bumper)
        {
            robot.spinnyWheel.setDirection(DcMotorEx.Direction.REVERSE);
            robot.spinnyWheel.setPower(.6);
        }
        else
        {
            robot.spinnyWheel.setPower(0.0);
            robot.spinnyWheel.setPower(0.0);
        }
    }

    private double getUpwardPower(int currentPosition)
    {
        return -.00006 * currentPosition * currentPosition + currentPosition * .006 + .35;
    }

    private double getDownwardPower(int currentPosition) {
        return -.000026 * currentPosition * currentPosition + currentPosition * .0034 - .08;
    }
}
