package org.firstinspires.ftc.team16911.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

@TeleOp(name = "Test")

public class Test extends OpMode
{
    // Standard Variables
    RigatoniHardware hardware;
    DcMotorEx[] motors;
    int maxPosition= 50;

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
        // Set motors to Run in Right Direction
        hardware.leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        hardware.leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        hardware.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        hardware.rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        motors = motors = new DcMotorEx[]{hardware.leftFront, hardware.leftRear,
                hardware.rightFront, hardware.rightRear};;

        // Sets ZeroPowerBehavior
        for (DcMotorEx motor : motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addData("Status", "Started");
        telemetry.update();
    }

    public void loop()
    {

        // Runs driver controlled code when not busy
        if (!isBusy())
        {
            for (DcMotorEx motor : motors)
            {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(gamepad1.left_stick_y);
            }
        }

        // Runs to highest position
        if (gamepad1.triangle)
        {
            telemetry.addData("Status", "Pressed");
            telemetry.update();


            for (DcMotorEx motor : motors)
            {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setTargetPosition(maxPosition);
            }

            for (DcMotorEx motor : motors)
            {
                motor.setPower(.1);
                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("Status", "Run");
            telemetry.update();
        }
    }

    private boolean isBusy()
    {
        return hardware.leftFront.isBusy() || hardware.leftRear.isBusy() ||
                hardware.rightFront.isBusy() || hardware.rightRear.isBusy();
    }
}
