package org.firstinspires.ftc.team15021.auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

public class util
{
    private RavioliHardware ravioliHardware;

    public util (RavioliHardware ravioliHardware)
    {
        this.ravioliHardware = ravioliHardware;
    }

    public void moveArm(int position)
    {
        ravioliHardware.armMotorOne.setTargetPosition(position);
        ravioliHardware.armMotorTwo.setTargetPosition(position);

        ravioliHardware.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ravioliHardware.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ravioliHardware.armMotorOne.setPower(.5);
        ravioliHardware.armMotorTwo.setPower(.5);
    }

    public void openClaw()
    {
        ravioliHardware.servoOne.setPosition(.5);
    }

    public void closeClaw()
    {
        ravioliHardware.servoOne.setPosition(0);
    }

    public void spinCarousel()
    {
        ravioliHardware.carouselMotorOne.setPower(.45);
        wait(3500);
        ravioliHardware.carouselMotorOne.setPower(0);
    }
    public void spinRedCarousel()
    {
        ravioliHardware.carouselMotorOne.setPower(-.45);
        wait(3500);
        ravioliHardware.carouselMotorOne.setPower(0);
    }

    public void wait(int waitTime)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (time.time() < waitTime)
        {
            continue;
        }
    }
}
