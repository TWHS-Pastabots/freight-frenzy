package org.firstinspires.ftc.team16912.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.firstinspires.ftc.team16912.util.Util;
import org.firstinspires.ftc.team16912.util.LiftBotHardware;
@TeleOp (name = "LiftBot")
public class LiftBot extends OpMode
{
    LiftBotHardware liftBotHardware;
    public void init()
    {
        liftBotHardware = new LiftBotHardware();
        liftBotHardware.init(hardwareMap);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("Status: ", "Started");
        telemetry.update();
    }

    public void loop()
    {
        if (gamepad1.right_trigger > 0.0)
        {
            liftBotHardware.liftMotorOne.setPower(gamepad1.right_trigger * .2);
        }
        else if (gamepad1.left_trigger > 0.0)
        {
            liftBotHardware.liftMotorOne.setPower(-gamepad1.left_trigger * .2);
        }
    }

    public void stop()
    {
        telemetry.addData("Status: ", "Started");
        telemetry.update();
    }
}
