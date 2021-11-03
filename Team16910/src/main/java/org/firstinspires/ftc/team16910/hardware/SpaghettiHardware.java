package org.firstinspires.ftc.team16910.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

public class SpaghettiHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx spinnyWheel = null;
    public DcMotorEx armMotorOne = null;
    public DcMotorEx armMotorTwo = null;
    public DcMotorEx[] motors;


    public void init(HardwareMap hardwareMap)
    {
        // Maps Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, SpaghettiIds.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, SpaghettiIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, SpaghettiIds.RIGHT_REAR_MOTOR);

        // Supplementary Motors
        spinnyWheel = hardwareMap.get(DcMotorEx.class, SpaghettiIds.SPINNY_WHEEL);
        armMotorOne = hardwareMap.get(DcMotorEx.class, SpaghettiIds.ARM_MOTOR_ONE);
        armMotorTwo = hardwareMap.get(DcMotorEx.class, SpaghettiIds.ARM_MOTOR_TWO);

        // Set Motors to Run in Right Direction
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear, spinnyWheel};

        // Set Zero Power Behavior and Initialize
        for (DcMotorEx motor : motors)
        {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void initializeArmMotors(HardwareMap hardwareMap)
    {
        // Maps Arm Motors
        armMotorOne = hardwareMap.get(DcMotorEx.class, SpaghettiIds.ARM_MOTOR_ONE);
        armMotorTwo = hardwareMap.get(DcMotorEx.class, SpaghettiIds.ARM_MOTOR_TWO);

        // Sets ZeroPowerBehavior
        armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set Motors to Run in Right Direction
        armMotorOne.setDirection(DcMotorEx.Direction.REVERSE);
        armMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset Arm Motors
        armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

}
