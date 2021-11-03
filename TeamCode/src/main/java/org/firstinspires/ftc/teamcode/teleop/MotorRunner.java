package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorRunner extends OpMode
{
    private DcMotorEx motor0 = null;
    private DcMotorEx motor1 = null;
    private DcMotorEx motor2 = null;
    private DcMotorEx motor3 = null;

    public void init()
    {
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");

        telemetry.addData("Motor0", motor0 != null);
        telemetry.update();
    }

    public void start()
    {
        telemetry.addData("Stats:", "Started");
        telemetry.update();
    }

    public void loop()
    {

    }

    public void stop()
    {

    }
}

