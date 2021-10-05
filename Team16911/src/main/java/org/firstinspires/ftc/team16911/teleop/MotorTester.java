package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MotorTester")
public class MotorTester extends OpMode
{
    private DcMotorEx motor0 = null;
    private DcMotorEx motor1 = null;
    private DcMotorEx motor2 = null;
    private DcMotorEx motor3 = null;
    private DcMotorEx[] motors = new DcMotorEx[4];

    public void init()
    {
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
        initializeMotors();
        runMotors();
        reverseDirection();
    }

    public void stop()
    {
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private void initializeMotors()
    {
        if (motor0 == null && gamepad1.dpad_up)
        {
            motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
            motors = new DcMotorEx[]{motor0, motor1, motor2, motor3};
        }
        if (motor1 == null && gamepad1.dpad_right)
        {
            motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
            motors = new DcMotorEx[]{motor0, motor1, motor2, motor3};
        }
        if (motor2 == null && gamepad1.dpad_down)
        {
            motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
            motors = new DcMotorEx[]{motor0, motor1, motor2, motor3};
        }
        if (motor3 == null && gamepad1.dpad_left)
        {
            motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
            motors = new DcMotorEx[]{motor0, motor1, motor2, motor3};
        }
    }

    private void runMotors()
    {
        if (motor0 != null && gamepad1.triangle)
        {
            motor0.setPower(1);
        }
        if (motor1 != null && gamepad1.circle)
        {
            motor1.setPower(1);
        }
        if (motor2 != null && gamepad1.cross)
        {
            motor2.setPower(1);
        }
        if (motor3 != null && gamepad1.square)
        {
            motor3.setPower(1);
        }
        for (DcMotorEx motor : motors)
        {
            if (motor != null)
            {
                motor.setPower(0);
            }
        }
    }

    private void reverseDirection()
    {
        if (gamepad1.right_bumper)
        {
            for (DcMotorEx motor : motors)
            {
                if (motor != null)
                {
                    motor.setDirection(DcMotorSimple.Direction.REVERSE);
                }
            }
        }
        else
        {
            for (DcMotorEx motor : motors)
            {
                if (motor != null)
                {
                    motor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }
        }
    }
}
