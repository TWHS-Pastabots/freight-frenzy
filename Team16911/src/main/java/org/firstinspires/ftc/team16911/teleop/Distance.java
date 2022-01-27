package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "Distance")
public class Distance extends OpMode
{
    RigatoniHardware hardware;
    final int TARGET_DISTANCE = 16;
    final double SLOW_CONSTANT = .6;
    final int MAX_ERROR = 2;
    final int MID_ERROR = 1;
    boolean automaticStrafe = false;

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
        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop()
    {
        double leftDistance = hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH);
        double rightDistance = hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH);
        double averageDistance = (leftDistance + rightDistance) / 2;
        double powerA = 0;
        double powerB = 0;

        if (gamepad1.circle)
        {
            telemetry.addData("Left Distance", leftDistance);
            telemetry.addData("Right Distance", rightDistance);
            telemetry.update();
        }
    }

    public void stop()
    {

    }
}
