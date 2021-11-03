package org.firstinspires.ftc.team16911.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

public class RigatoniHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;
    public DcMotorEx armMotorOne = null;
    public DcMotorEx armMotorTwo = null;
    public DcMotorEx carouselMotorOne = null;
    public DcMotorEx carouselMotorTwo = null;
    public DcMotorEx[] motors;

    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        initializePrimaryMotors(hardwareMap);
        initializeArmMotors(hardwareMap);
        initializeCarousel(hardwareMap);
    }

    private void initializePrimaryMotors(HardwareMap hardwareMap)
    {
        // Maps Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_REAR_MOTOR);

        // Set Motors to Run in Right Direction
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear};

        // Set Zero Power Behavior and Initialize
        for (DcMotorEx motor : motors)
        {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void initializeArmMotors(HardwareMap hardwareMap)
    {
        // Maps Arm Motors
        armMotorOne = hardwareMap.get(DcMotorEx.class, RigatoniIds.ARM_MOTOR_ONE);
        armMotorTwo = hardwareMap.get(DcMotorEx.class, RigatoniIds.ARM_MOTOR_TWO);

        // Sets ZeroPowerBehavior
        armMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set Motors to Run in Right Direction
        armMotorOne.setDirection(DcMotorEx.Direction.REVERSE);
        armMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);

        // Reset Arm Motors
        armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void initializeCarousel(HardwareMap hardwareMap)
    {
        carouselMotorOne = hardwareMap.get(DcMotorEx.class, RigatoniIds.CAROUSEL_MOTOR_ONE);
        carouselMotorTwo = hardwareMap.get(DcMotorEx.class, RigatoniIds.CAROUSEL_MOTOR_TWO);

        carouselMotorOne.setPower(0.0);
        carouselMotorTwo.setPower(0.0);

        carouselMotorOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        carouselMotorTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        carouselMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
