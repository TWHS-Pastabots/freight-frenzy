package org.firstinspires.ftc.team16911.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;
import org.firstinspires.ftc.team16911.hardware.RigatoniIds;

@TeleOp(name = "Test")
public class Test extends OpMode
{
    // Standard Variables
    RigatoniHardware hardware;

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
        hardware.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop()
    {
        // Carousel Code
        if (gamepad1.right_bumper)
        {
            hardware.leftFront.setPower(.4);
        }
        else
        {
            hardware.leftFront.setPower(0);
        }
    }


/*      Initial prototype Code for Arm
        // Runs driver controlled code when not busy
        if (!hardware.leftFront.isBusy())
        {
            hardware.leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad1.right_trigger > 0)
            {
                hardware.leftFront.setPower(gamepad1.right_trigger * .5);
            }
            else
            {
                hardware.leftFront.setPower(-gamepad1.left_trigger * .5);
            }
        }

        // Runs to highest position
        if (gamepad1.triangle)
        {
            telemetry.addData("Status", "Pressed");
            telemetry.update();

            // Moves to maximum position
            hardware.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hardware.leftFront.setTargetPosition(maxPosition);
            hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.leftFront.setPower(.3);

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
            hardware.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }*/
}
