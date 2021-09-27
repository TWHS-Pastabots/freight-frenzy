package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode
{
    RigatoniHardware hardware = new RigatoniHardware();

    public void init()
    {
        hardware.init(hardwareMap);
        hardware.servoTest.scaleRange(0.0, 1.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("Status", "Started");
        telemetry.update();

        hardware.servoTest.setPwmEnable();
        telemetry.addData("Status", hardware.servoTest.isPwmEnabled());
        telemetry.update();
    }

    public void loop()
    {
        // Goes to max position
        if (gamepad1.triangle)
        {
            hardware.servoTest.setPosition(1.0);
            telemetry.addData("Status", hardware.servoTest.getPosition());
            telemetry.update();
        }

        //Goes to half position
        if (gamepad1.circle)
        {
            hardware.servoTest.setPosition(.5);
            telemetry.addData("Status", hardware.servoTest.getPosition());
            telemetry.update();
        }

        if (gamepad1.cross)
        {
            hardware.servoTest.setPosition(0);
            telemetry.addData("Status", hardware.servoTest.getPosition());
            telemetry.update();
        }
    }
}
