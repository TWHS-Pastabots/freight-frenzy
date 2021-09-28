package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;
import org.firstinspires.ftc.team16911.hardware.TestHardware;

@TeleOp(name = "EncoderTestDrive")
public class EncoderTestDrive extends OpMode
{
    // Standard Variables
    TestHardware hardware;
    DcMotorEx[] motors;
    int maxPosition= 2300;

    public void init()
    {
        // Initialize Hardware
        hardware = new TestHardware();
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
        for (DcMotorEx motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop() {

        // Runs driver controlled code when not busy
        if (!hardware.leftFront.isBusy() && !hardware.rightFront.isBusy() &&
                !hardware.leftRear.isBusy() && !hardware.rightRear.isBusy())
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
        }

        // Runs to highest position
        if (gamepad1.triangle)
        {
            telemetry.addData("Status", "Pressed");
            telemetry.update();

            // Moves to maximum position
            for (DcMotorEx motor : motors)
            {
                motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                motor.setTargetPosition(maxPosition);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setVelocity(1000);
            }

            telemetry.addData("Status", "Run");
            telemetry.update();
        }

        // Displays current position
        if (gamepad1.circle)
        {
            telemetry.addData("Current Position", hardware.leftFront.getCurrentPosition());
            telemetry.update();
        }

        if (gamepad1.dpad_down)
        {
            for (DcMotorEx motor : motors)
            {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
}
