package org.firstinspires.ftc.team16911.hardware;

import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.LEFT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.LEFT_REAR_MOTOR;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.RIGHT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.RIGHT_REAR_MOTOR;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

import java.util.ArrayList;
import java.util.Arrays;

public class RigatoniHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx[] motors;

    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        // Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_MOTOR);
        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear};

        // Set Zero Power Behavior and Initialize
        for (DcMotorEx motor : motors)
        {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
