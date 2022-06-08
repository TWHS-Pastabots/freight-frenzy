package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team16911.autonomous.Utilities;

public class Anders extends OpMode
{
    DcMotorEx motor0;

    public void init()
    {
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
    }

    public void loop()
    {
        if (gamepad1.triangle)
        {
            motor0.setPower(1);
        }
        else
        {
            motor0.setPower(0);
        }
    }
}
