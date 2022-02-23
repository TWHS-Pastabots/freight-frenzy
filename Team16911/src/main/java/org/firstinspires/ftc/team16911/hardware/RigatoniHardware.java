package org.firstinspires.ftc.team16911.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    public DcMotorEx carouselMotor = null;
    public DcMotorEx intakeMotor = null;
    public CRServo cappingServo = null;
    public DistanceSensor leftDistanceSensor = null;
    public DistanceSensor rightDistanceSensor = null;
    public DistanceSensor backDistanceSensor = null;
    public DcMotorEx[] motors;

    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        initializePrimaryMotors(hardwareMap);
        initializeArmMotors(hardwareMap);
        initializeCarousel(hardwareMap);
        initializeIntakeMotor(hardwareMap);
        initializeDistanceSensors(hardwareMap);
        initializeCappingServo(hardwareMap);
    }

    private void initializePrimaryMotors(HardwareMap hardwareMap)
    {
        // Maps Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_REAR_MOTOR);

        // Set Motors to Run in Right Direction
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);

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
        armMotorOne.setDirection(DcMotorEx.Direction.FORWARD);
        armMotorTwo.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset Arm Motors
        armMotorOne.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorOne.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorTwo.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotorOne.setTargetPosition(0);
        armMotorTwo.setTargetPosition(0);
    }

    private void initializeCarousel(HardwareMap hardwareMap)
    {
        carouselMotor = hardwareMap.get(DcMotorEx.class, RigatoniIds.CAROUSEL_MOTOR);

        carouselMotor.setPower(0.0);

        carouselMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        carouselMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeIntakeMotor(HardwareMap hardwareMap)
    {
        intakeMotor = hardwareMap.get(DcMotorEx.class, RigatoniIds.INTAKE_MOTOR);

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    private void initializeDistanceSensors(HardwareMap hardwareMap)
    {
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, RigatoniIds.LEFT_DISTANCE_SENSOR);
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, RigatoniIds.RIGHT_DISTANCE_SENSOR);
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, RigatoniIds.BACK_DISTANCE_SENSOR);
    }

    private void initializeCappingServo(HardwareMap hardwareMap)
    {
        cappingServo = hardwareMap.get(CRServo.class, RigatoniIds.CAPPING_SERVO);
        cappingServo.setDirection(CRServo.Direction.FORWARD);
        cappingServo.setPower(0);
    }
}
