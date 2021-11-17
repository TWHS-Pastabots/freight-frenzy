package org.firstinspires.ftc.team16909.hardware;

import static org.firstinspires.ftc.team16909.hardware.FettuccineIds.ARM_ENCODER;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.LEFT_ENCODER;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.LEFT_REAR_MOTOR;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.RIGHT_FRONT_MOTOR;
import static org.firstinspires.ftc.teamcode.hardware.ComponentIds.RIGHT_REAR_MOTOR;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

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

    // Supplemental DC Motors
    public DcMotorEx carousel = null;
    public DcMotorEx leftArm = null;
    public DcMotorEx rightArm = null;

    // Supplemental Servo Motors
    public CRServo armServo = null;
    public Servo grabber = null;


    public DcMotorEx[] motors;

    public ServoImplEx servoTest = null;

    // Supplemental Encoders
    public Encoder encoderArm = null;


    public void init(HardwareMap hardwareMap)
    {
        Assert.assertNotNull(hardwareMap);

        // Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, FettuccineIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, FettuccineIds.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, FettuccineIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, FettuccineIds.RIGHT_REAR_MOTOR);


        // Supplemental DC Motors
        carousel = hardwareMap.get(DcMotorEx.class, FettuccineIds.CAROUSEL);
        leftArm = hardwareMap.get(DcMotorEx.class, FettuccineIds.LEFT_ARM_MOTOR);
        rightArm = hardwareMap.get(DcMotorEx.class, FettuccineIds.RIGHT_ARM_MOTOR);

        // TEMP CODE
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);

        encoderArm = new Encoder(hardwareMap.get(DcMotorEx.class, ARM_ENCODER));

        // Supplemental Servo Motors
        armServo = hardwareMap.get(CRServo.class, FettuccineIds.ARM_SERVO);
        grabber = hardwareMap.get(Servo.class, FettuccineIds.GRABBER);


        // Supplemental Directions
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.REVERSE);


        grabber.setDirection(Servo.Direction.FORWARD);
        // Motor Types


        motors = new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear, carousel, leftArm, rightArm};

        // Set Zero Power Behavior and Initialize
        for (DcMotorEx motor : motors)
        {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
    }
}
