package org.firstinspires.ftc.team16911.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team16911.hardware.RigatoniHardware;

public class Utilities
{
    public final int[] positions = {123, 180, 220};
    public final int initialArmPosition = 150;
    public final int CARGO_DROP_TIME = 1200;
    public final double[] DROP_POWERS = {.725, .9, .725};
    private RigatoniHardware hardware;
    private final int MAX_TRIGGER_DISTANCE = 13;

    Utilities(RigatoniHardware hardware)
    {
        this.hardware = hardware;
    }

    public void dropCargo(int waitTime, double power, Telemetry telemetry)
    {
        hardware.intakeMotor.setPower(-power);
        wait(waitTime, telemetry);
        hardware.intakeMotor.setPower(0);
    }

    public void intakeCargo()
    {
        hardware.intakeMotor.setPower(.775);
    }

    public void stopIntake()
    {
        hardware.intakeMotor.setPower(0.0);
    }

    public void spinCarouselAndMoveArm(int waitTime, int position, Telemetry telemetry)
    {
        hardware.carouselMotor.setPower(.5);
        moveArm(position);
        wait(waitTime, telemetry);
        hardware.carouselMotor.setPower(0.0);
    }

    public void spinCarousel(int waitTime, Telemetry telemetry)
    {
        hardware.carouselMotor.setPower(.5);
        wait(waitTime, telemetry);
        hardware.carouselMotor.setPower(0.0);
    }

    public void moveArm(int position)
    {
        hardware.armMotorOne.setTargetPosition(position);
        hardware.armMotorTwo.setTargetPosition(position);

        hardware.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        hardware.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        hardware.armMotorOne.setPower(1);
        hardware.armMotorTwo.setPower(1);
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
