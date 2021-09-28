package org.firstinspires.ftc.team16911.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class TestHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx armMotor = null;
    public DcMotorEx[] motors;

    public ServoImplEx servoTest = null;
    public Encoder leftEncoder = null;
    public Encoder rightEncoder = null;
    public Encoder frontEncoder = null;


    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        // Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_Rear_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_REAR_MOTOR);
        //armMotor = hardwareMap.get(DcMotorEx.class, RigatoniIds.ARM_MOTOR);
        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear};

        // Test Code
        //servoTest = hardwareMap.get(ServoImplEx.class, RigatoniIds.SERVO_TEST);
        leftEncoder = (Encoder) hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_ENCODER);
        rightEncoder = (Encoder) hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_ENCODER);
        frontEncoder = (Encoder) hardwareMap.get(DcMotorEx.class, RigatoniIds.FRONT_ENCODER);

        // Set Zero Power Behavior and Initialize
        for (DcMotorEx motor : motors)
        {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}
