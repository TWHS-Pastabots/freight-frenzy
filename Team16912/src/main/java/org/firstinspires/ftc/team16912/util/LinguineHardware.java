package org.firstinspires.ftc.team16912.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.List;

public class LinguineHardware extends RobotHardware {

    public DcMotorEx motorArm1 = null;
    public DcMotorEx motorArm2 = null;

    public List<DcMotorEx> motorArms;

    public DcMotorEx cSpinner = null;

    public Servo servoClaw = null;

    public Encoder armEncoder = null;

    public CRServo tapeServo = null;
    public CRServo LRServo = null;
    public CRServo UDMotor = null;

    @Override
    public void init(HardwareMap hardwareMap, boolean shouldInitializeComponents) {
        super.init(hardwareMap, shouldInitializeComponents);

        // Prepare motor directions
        motorLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotorEx.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        motorRightRear.setDirection(DcMotorEx.Direction.FORWARD);

        for (DcMotorEx motor : wheels) motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Carousel Spinner
        cSpinner = hardwareMap.get(DcMotorEx.class, LinguineIds.MOTOR_SPINNER);
        cSpinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        cSpinner.setDirection(DcMotorEx.Direction.REVERSE);
        cSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cSpinner.setPower(.0);

        // Arm Motors
        motorArm1 = hardwareMap.get(DcMotorEx.class, LinguineIds.MOTOR_ARM_1);
        motorArm2 = hardwareMap.get(DcMotorEx.class, LinguineIds.MOTOR_ARM_2);

        motorArms = new ArrayList<>();
        motorArms.add(motorArm1);
        motorArms.add(motorArm2);


        motorArm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorArm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        servoClaw = hardwareMap.get(Servo.class, LinguineIds.SERVO_CLAW);

        UDMotor = hardwareMap.get(CRServo.class, LinguineIds.UD_MOTOR);
        LRServo = hardwareMap.get(CRServo.class, LinguineIds.LR_SERVO);
        tapeServo = hardwareMap.get(CRServo.class, LinguineIds.TAPE_SERVO);

        armEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LinguineIds.MOTOR_ARM_1));
//        UDMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
