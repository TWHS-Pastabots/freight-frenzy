package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;
import org.firstinspires.ftc.team16911.hardware.TestHardware;

@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode
{
    TestHardware hardware;

    public void init()
    {
        hardware = new TestHardware();
        hardware.init(hardwareMap);
        hardware.servoTest.setPwmRange(PwmControl.PwmRange.defaultRange);
        hardware.servoTest.setPwmEnable();

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
        // Goes to max position
        if (gamepad1.triangle)
        {
            hardware.servoTest.setPosition(1.0);
        }

        //Goes to half position
        if (gamepad1.circle)
        {
            hardware.servoTest.setPosition(.5);
        }
    }
}
