package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "Rigatoni")
public class Rigatoni extends OpMode
{
    RigatoniHardware hardware;
    int maxPosition = 300;

    public void init()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        // Set motors to Run in Right Direction
        hardware.leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        hardware.leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        hardware.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        hardware.rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop()
    {
        drive();
        //moveArm();
        //spinCarousel();
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
            hardware.armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.armMotor.setPower(gamepad1.right_trigger * .75);
        }
        else if (gamepad1.left_trigger > 0)
        {
            hardware.armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hardware.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.armMotor.setPower(-gamepad1.left_trigger * .75);
        }
        else
        {
            hardware.armMotor.setTargetPosition(hardware.armMotor.getCurrentPosition());
            hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.armMotor.setPower(1.0);
        }

        // Runs to highest position
        if (gamepad1.triangle)
        {
            hardware.armMotor.setTargetPosition(maxPosition);
            hardware.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.armMotor.setPower(.75);
        }

        // Resets zero position for calibration
        if (gamepad1.dpad_down)
        {
            hardware.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Displays current position for development purposes
        if (gamepad1.circle)
        {
            telemetry.addData("Current Position", hardware.armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void spinCarousel()
    {
        // Carousel Motor Code
        if (gamepad1.right_bumper)
        {
            hardware.carouselMotor.setPower(.4);
        }
        else
        {
            hardware.carouselMotor.setPower(0);
        }
    }
}
