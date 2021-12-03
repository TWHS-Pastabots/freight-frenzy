package org.firstinspires.ftc.team15021.hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

public class RavioliHardware
{
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx armMotorOne = null;
    public DcMotorEx armMotorTwo = null;
    public DcMotorEx[] motors;


    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        initializeDriveMotors(hardwareMap);
        initializeArmMotors(hardwareMap);
    }

    private void initializeDriveMotors(HardwareMap hardwareMap)
    {
        leftFront = hardwareMap.get(DcMotorEx.class, RavioliIds.LEFT_FRONT_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RavioliIds.RIGHT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, RavioliIds.LEFT_REAR_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RavioliIds.RIGHT_REAR_MOTOR);

        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear};

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

        for (DcMotorEx motor : motors)
        {
            motor.setPower(0.0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
    private void initializeArmMotors(HardwareMap hardwareMap)
    {
        // Maps Arm Motors
        armMotorOne = hardwareMap.get(DcMotorEx.class, RavioliIds.ARM_MOTOR_ONE);
        armMotorTwo = hardwareMap.get(DcMotorEx.class, RavioliIds.ARM_MOTOR_TWO);

        // Sets ZeroPowerBehavior
        armMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set Motors to Run in Right Direction
        armMotorOne.setDirection(DcMotorEx.Direction.FORWARD);
        armMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset Arm Motors
        armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
