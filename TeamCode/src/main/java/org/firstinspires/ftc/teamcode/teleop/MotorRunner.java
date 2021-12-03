package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MotorRunner")
public class MotorRunner extends OpMode
{
    boolean reversed = false;
    DcMotorEx motor0 = null;
    DcMotorEx motor1 = null;
    DcMotorEx motor2 = null;
    DcMotorEx motor3 = null;
    ElapsedTime reverseTime = null;
    DcMotorEx motors [];
    public double speed = .75;

    public void init()
    {
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");

        motors = new DcMotorEx[] {motor0, motor1, motor2, motor3};

        reverseTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        telemetry.addData("gamer", "Initialized!");
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("gamer","Started!");
        telemetry.update();
    }

    public void loop()
    {
        for (DcMotorEx motor : motors)
        {
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad1.right_bumper && !reversed && reverseTime.time()>=300)
        {
            speed = -speed;
            reversed = true;
            reverseTime.reset();
        }
        else if(gamepad1.right_bumper && reversed && reverseTime.time()>=300)
        {
            speed = -speed;
            reversed = false;
            reverseTime.reset();
        }

        if(gamepad1.dpad_up)
        {
            for(DcMotorEx motor : motors)
            {
                motor.setMode((DcMotorEx.RunMode.RUN_USING_ENCODER));

            }
        }
        if (gamepad1.triangle)
        {
            motor0.setPower(speed);
        }
        else
        {
            motor0.setPower(0);
        }

        if (gamepad1.circle)
        {
            motor1.setPower(speed);
        }
        else
        {
            motor1.setPower(0);
        }

        if (gamepad1.cross)
        {
            motor2.setPower(speed);
        }
        else
        {
            motor2.setPower(0);
        }

        if (gamepad1.square)
        {
            motor3.setPower(speed);
        }
        else
        {
            motor3.setPower(0);
        }

        telemetry.addData("0currentPos:", motor0.getCurrentPosition());
        telemetry.addData("1currentPos:", motor1.getCurrentPosition());
        telemetry.addData("2currentPos:", motor2.getCurrentPosition());
        telemetry.addData("3currentPos:", motor3.getCurrentPosition());
        telemetry.update();

    }

    public void stop()
    {
        telemetry.addData("gamer", "Stopped :(");
        telemetry.update();
    }
}
