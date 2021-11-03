package org.firstinspires.ftc.team15021.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@TeleOp (name = "EncoderExample")
public class EncoderExample extends OpMode
{
    RavioliHardware hardware;
    int maxPosition = 2000;

    public void init()
    {
        hardware = new RavioliHardware();
        hardware.init(hardwareMap);
        for (DcMotorEx motor : hardware.motors)
        {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Initialized");
    }

    public void start()
    {
        for (DcMotorEx motor: hardware.motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addLine("Started");
    }

    public void loop()
    {
        if (gamepad1.right_trigger > 0)
        {
            for (DcMotorEx motor : hardware.motors)
            {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motor.setPower(gamepad1.right_trigger);
            }
        }
        else if (gamepad1.left_trigger > 0)
        {
            for (DcMotorEx motor : hardware.motors)
            {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motor.setPower(-gamepad1.left_trigger);
            }
        }
        else if (!hardware.leftFront.isBusy())
        {
            for (DcMotorEx motor : hardware.motors)
            {
                motor.setPower(0);
            }
        }

        if (gamepad1.triangle)
        {
            for (DcMotorEx motor : hardware.motors)
            {
                motor.setTargetPosition(maxPosition);
                
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                motor.setPower(1.0);
            }
        }

        if (gamepad1.cross)
        {
            for (DcMotorEx motor : hardware.motors)
            {
                motor.setTargetPosition(0);

                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                motor.setPower(1.0);
            }
        }

        if (gamepad1.dpad_down)
        {
            for (DcMotorEx motor : hardware.motors)
            {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        if (gamepad1.circle)
        {
            telemetry.addData("Current Position", hardware.leftFront.getCurrentPosition());
        }

    }

    public void stop()
    {
        telemetry.addLine("Stopped");
    }
}
