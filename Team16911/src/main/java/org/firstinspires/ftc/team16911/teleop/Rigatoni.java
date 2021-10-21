package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "Rigatoni")
public class Rigatoni extends OpMode
{
    RigatoniHardware hardware;
    int maxPosition = 100;
    boolean justMoved = false;

    public void init()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);

        telemetry.addData("Status", "Newest Code");
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
        //moveArm();
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

        hardware.leftFront.setPower(leftFrontPower);
        hardware.leftRear.setPower(leftRearPower);
        hardware.rightFront.setPower(rightFrontPower);
        hardware.rightRear.setPower(rightRearPower);
    }

    private void moveArm()
    {
        // Runs driver controlled code
        if (gamepad1.right_trigger > 0)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            hardware.armMotorOne.setPower(gamepad1.right_trigger * .5);
            hardware.armMotorTwo.setPower(gamepad1.right_trigger * .5);
            justMoved = true;
        }
        else if (gamepad1.left_trigger > 0)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            hardware.armMotorOne.setPower(getDownwardPower(hardware.armMotorOne.getCurrentPosition()));
            hardware.armMotorTwo.setPower(getDownwardPower(hardware.armMotorTwo.getCurrentPosition()));

            justMoved = true;
        }
        else if (justMoved)
        {
            hardware.armMotorOne.setTargetPosition(hardware.armMotorOne.getCurrentPosition());
            hardware.armMotorTwo.setTargetPosition(hardware.armMotorTwo.getCurrentPosition());

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            hardware.armMotorOne.setPower(1.0);
            hardware.armMotorTwo.setPower(1.0);

            justMoved = false;
        }

        // Runs to highest position
        if (gamepad1.triangle)
        {
            hardware.armMotorOne.setTargetPosition(maxPosition);
            hardware.armMotorTwo.setTargetPosition(maxPosition);

            hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            hardware.armMotorOne.setPower(.5);
            hardware.armMotorTwo.setPower(.5);
            justMoved = false;
        }

        // Resets zero position for calibration
        if (gamepad1.dpad_down)
        {
            hardware.armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hardware.armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Displays current position for development purpose
        if (gamepad1.circle)
        {
            telemetry.addData("Current Position", hardware.armMotorOne.getCurrentPosition());
            telemetry.update();
        }
        else if (gamepad1.square)
        {
            telemetry.addData("Current Position", hardware.armMotorTwo.getCurrentPosition());
            telemetry.update();
        }
    }

    private void spinCarousel()
    {
        // Carousel Motor Code
        if (gamepad1.right_bumper)
        {
            hardware.carouselMotor.setDirection(DcMotorEx.Direction.FORWARD);
            hardware.carouselMotor.setPower(.4);
        }
        else if (gamepad1.left_bumper)
        {
            hardware.carouselMotor.setDirection(DcMotorEx.Direction.REVERSE);
            hardware.carouselMotor.setPower(.4);
        }
        else
        {
            hardware.carouselMotor.setPower(0.0);
        }
    }

    private double getDownwardPower(int currentPosition)
    {
        return -.000014 * currentPosition * currentPosition + currentPosition * .0016 - .02;
    }
}
