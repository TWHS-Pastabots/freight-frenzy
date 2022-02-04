package org.firstinspires.ftc.team15021.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "EncoderWireTester")
public class EncoderWireTester extends OpMode
{
    DcMotorEx motor0;
    DcMotorEx motor1;
    ElapsedTime time;
    ElapsedTime buttonTime;
    int timeToRun = 3000;
    public void init()
    {
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");

        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);

        motor0.setPower(0.0);
        motor1.setPower(0.0);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("Status","Initialized");
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("Status","Started");
        telemetry.update();
    }

    public void loop()
    {
        if(gamepad1.left_bumper)
        {
            motor0.setDirection(DcMotorSimple.Direction.REVERSE);
            motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if(gamepad1.right_bumper)
        {
            motor0.setDirection(DcMotorSimple.Direction.FORWARD);
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if(gamepad1.dpad_up && buttonTime.time() >= 500)
        {
            timeToRun += 1000;
            buttonTime.reset();
        }
        else if(gamepad1.dpad_down && buttonTime.time() >= 500)
        {
            timeToRun -= 1000;
            buttonTime.reset();
        }

        if (gamepad1.circle)
        {
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad1.cross)
        {
            time.reset();
        }

        if (time.time() < timeToRun)
        {
            motor0.setPower(1.0);
            motor1.setPower(1.0);
        }
        else
        {
            motor0.setPower(0);
            motor1.setPower(0);
        }

        telemetry.addData("Time", time.time());
        telemetry.addData("Time to Run", timeToRun);
        telemetry.addData("Motor 0 Position", motor0.getCurrentPosition());
        telemetry.addData("Motor 1 Position", motor1.getCurrentPosition());
        telemetry.addData("Motor 0 Direction", motor0.getDirection());
        telemetry.addData("Motor 1 Direction", motor1.getDirection());
        telemetry.update();
    }

    public void stop()
    {
        telemetry.addData("Status","Stopped");
        telemetry.update();
    }

}
