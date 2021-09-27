package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "EncoderTestDrive")
public class EncoderTestDrive extends OpMode
{
    // Standard Variables
    RigatoniHardware hardware;
    DcMotorEx[] motors;
    int targetPosition = 2000;

    public void init()
    {
        // Initialize Hardware
        hardware = new RigatoniHardware();
        hardware.init(hardwareMap);
        motors = new DcMotorEx[]{hardware.leftFront, hardware.rightFront, hardware.leftRear, hardware.rightRear};

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        hardware.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        hardware.leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        hardware.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        hardware.rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop()
    {
        for (DcMotorEx motor : motors)
        {
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad1.right_trigger > 0)
            {
                motor.setPower(.5 * gamepad1.right_trigger);
            }
            else
            {
                motor.setPower(.5 * -gamepad1.left_trigger);
            }
        }

        // Runs to highest position
        if (gamepad1.triangle)
        {
            telemetry.addData("Status", "Pressed");
            telemetry.update();

            while (!reachedPos(targetPosition))
            {
                double y = targetPosition - hardware.encoderTest.getCurrentPosition();
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
            telemetry.addData("Status", "Run");
            telemetry.update();
        }

        // Displays current position
        if (gamepad1.circle)
        {
            telemetry.addData("Current Position", hardware.encoderTest.getCurrentPosition());
            telemetry.update();
        }
    }

    private Boolean reachedPos(int targetPosition)
    {
        return hardware.encoderTest.getCurrentPosition() == targetPosition;
    }
}
