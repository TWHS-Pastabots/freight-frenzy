package org.firstinspires.ftc.team16909.hardware;

import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.LEFT_REAR_MOTOR;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.RIGHT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.RIGHT_REAR_MOTOR;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.ComponentIds;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;

public class FettuccineHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    // Supplemental Motors
    public DcMotorEx carousel = null;


    public DcMotorEx[] motors;

    public ServoImplEx servoTest = null;
    public Encoder encoderTest = null;


    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        // Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, FettuccineIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, FettuccineIds.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, FettuccineIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, FettuccineIds.RIGHT_REAR_MOTOR);

        // Supplemental Motors
        carousel = hardwareMap.get(DcMotorEx.class, FettuccineIds.CAROUSEL);

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
