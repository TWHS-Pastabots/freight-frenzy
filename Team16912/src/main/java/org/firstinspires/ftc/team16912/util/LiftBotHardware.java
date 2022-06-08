package org.firstinspires.ftc.team16912.util;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.firstinspires.ftc.team16912.util.Util;


public class LiftBotHardware
{
    public DcMotorEx liftMotorOne;

    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        initializeLift(hardwareMap);
    }

    public void initializeLift(HardwareMap hardwareMap)
    {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, LinguineIds.LIFT_MOTOR_ONE);

        liftMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorOne.setPower(0.0);
        liftMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        liftMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
