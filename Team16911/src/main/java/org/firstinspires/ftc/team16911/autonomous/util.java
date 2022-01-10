package org.firstinspires.ftc.team16911.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

public class util
{
    private RigatoniHardware hardware;
    private final int MAX_TRIGGER_DISTANCE = 10;

    util(RigatoniHardware hardware)
    {
        this.hardware = hardware;
    }

    public void dropCargo(int waitTime, Telemetry telemetry)
    {
        hardware.armServo.setPower(-1);
        wait(waitTime, telemetry);
        hardware.armServo.setPower(0);
    }

    public void spinCarouselAndMoveArm(int waitTime, int position, Telemetry telemetry)
    {
        hardware.carouselMotorOne.setPower(.5);
        hardware.carouselMotorTwo.setPower(.5);
        moveArm(position);
        wait(waitTime, telemetry);
        hardware.carouselMotorOne.setPower(0.0);
        hardware.carouselMotorTwo.setPower(0.0);
    }

    public void spinCarousel(int waitTime, Telemetry telemetry)
    {
        hardware.carouselMotorOne.setPower(.5);
        hardware.carouselMotorTwo.setPower(.5);
        wait(waitTime, telemetry);
        hardware.carouselMotorOne.setPower(0.0);
        hardware.carouselMotorTwo.setPower(0.0);
    }

    public void moveArm(int position)
    {
        hardware.armMotorOne.setTargetPosition(position);
        hardware.armMotorTwo.setTargetPosition(position);

        hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        hardware.armMotorOne.setPower(.8);
        hardware.armMotorTwo.setPower(.8);
    }

    public void wait(int waitTime, Telemetry telemetry)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time() < waitTime)
        {
            telemetry.addData("Status", "Waiting");
            telemetry.addData("Wait Time", waitTime / 1000);
            telemetry.addData("Time Left", (waitTime - time.time()) / 1000);
            telemetry.update();
            continue;
        }
    }

    public void eliminateOscillations()
    {
        hardware.armMotorOne.setTargetPosition(hardware.armMotorOne.getCurrentPosition());
        hardware.armMotorTwo.setTargetPosition(hardware.armMotorTwo.getCurrentPosition());
    }

    public int getBarcodeLevelBlueSide()
    {
        if (hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= MAX_TRIGGER_DISTANCE)
        {
            return 2;
        }
        else if (hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH) <= MAX_TRIGGER_DISTANCE)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    public int getBarcodeLevelRedSide()
    {
        if (hardware.rightDistanceSensor.getDistance(DistanceUnit.INCH) <= MAX_TRIGGER_DISTANCE)
        {
            return 1;
        }
        else if (hardware.leftDistanceSensor.getDistance(DistanceUnit.INCH) <= MAX_TRIGGER_DISTANCE)
        {
            return 0;
        }
        else
        {
            return 2;
        }
    }
}
