package org.firstinspires.ftc.team16910.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class SpaghettiHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    // Supplemental Motors
    public DcMotorEx spinnyWheel = null;
    public DcMotorEx leftArmMotor = null;
    public DcMotorEx rightArmMotor = null;


    public DcMotorEx[] motors;

    public ServoImplEx servoTest = null;
    public Encoder encoderTest = null;


    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        // Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, SpaghettiIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, SpaghettiIds.RIGHT_REAR_MOTOR);

        // Supplemental Motors
        spinnyWheel = hardwareMap.get(DcMotorEx.class, SpaghettiIds.SPINNY_WHEEL);
        leftArmMotor = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LEFT_ARM_MOTOR);
        rightArmMotor= hardwareMap.get(DcMotorEx.class, SpaghettiIds.RIGHT_ARM_MOTOR);

        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear, spinnyWheel, leftArmMotor, rightArmMotor};

        // Set Zero Power Behavior and Initialize
        for (DcMotorEx motor : motors)
        {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
