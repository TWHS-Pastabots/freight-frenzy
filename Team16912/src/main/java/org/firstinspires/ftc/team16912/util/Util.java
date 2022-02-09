package org.firstinspires.ftc.team16912.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Util {

    static LinguineHardware robot = new LinguineHardware();
    static SampleMecanumDrive drive;
    static ElapsedTime runTime = new ElapsedTime();

    public static void init(HardwareMap hwMap) {
        robot.init(hwMap);
        drive = new SampleMecanumDrive(hwMap);
    }


    // Closes claw
    public static void closeClaw(double seconds) {
        double initTime = runTime.seconds();
        while (runTime.seconds() - initTime < seconds) robot.servoClaw.setPosition(.7);
    }

    // Opens claw
    public static void openClaw(double seconds) {
        double initTime = runTime.seconds();
        while (runTime.seconds() - initTime < seconds) robot.servoClaw.setPosition(.1);
    }

    public static void runArmTo(int targetPos)
    {
        if(robot.armEncoder.getCurrentPosition() < targetPos)
        {
            while(robot.armEncoder.getCurrentPosition() < targetPos)
            {
                for (DcMotorEx motor : robot.motorArms) {
                    motor.setPower(.5);
                }
            }
        }
        else if(robot.armEncoder.getCurrentPosition() > targetPos)
        {
            while(robot.armEncoder.getCurrentPosition() > targetPos)
            {
                for (DcMotorEx motor : robot.motorArms) {
                    motor.setPower(-.5);
                }
            }
        }
        for (DcMotorEx motor : robot.motorArms) motor.setPower(0);
    }

    public static void setSpinnerDirection(char dir) {

        if (dir == 'f') robot.cSpinner.setDirection(DcMotorSimple.Direction.FORWARD);

        else if (dir == 'r') robot.cSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
    }


}
