package org.firstinspires.ftc.team16912.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;
import java.util.List;

public class LinguineHardware extends RobotHardware {

    public DcMotorEx motorArm1 = null;
    public DcMotorEx motorArm2 = null;

    public List<DcMotorEx> motorArms;

    public DcMotorEx cSpinner = null;

    @Override
    public void init(HardwareMap hardwareMap, boolean shouldInitializeComponents) {
        super.init(hardwareMap, shouldInitializeComponents);

        // Carousel Spinner
        cSpinner = hardwareMap.get(DcMotorEx.class, LinguineIds.MOTOR_SPINNER);
        cSpinner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        cSpinner.setPower(.0);

        // Arm Motors
        motorArm1 = hardwareMap.get(DcMotorEx.class, LinguineIds.MOTOR_ARM_1);
        motorArm2 = hardwareMap.get(DcMotorEx.class, LinguineIds.MOTOR_ARM_2);

        motorArms = new ArrayList<>();
        motorArms.add(motorArm1);
        motorArms.add(motorArm2);

        for (DcMotorEx motor : motorArms) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

}
