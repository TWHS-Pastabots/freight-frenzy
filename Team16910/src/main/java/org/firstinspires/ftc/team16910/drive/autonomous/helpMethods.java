package org.firstinspires.ftc.team16910.drive.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;

public class helpMethods
{
    private SpaghettiHardware robot;

    boolean grab = false;
    boolean justMoved = false;
    static ElapsedTime armTime = null;
    ElapsedTime lockout = null;
    ElapsedTime timeSinceMove = null;

    // Arm Values
    static int currentPosition = 0;
    static int leftArmOffset = 0;

    helpMethods(SpaghettiHardware robot)
    {
        this.robot = robot;

        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeSinceMove = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        lockout = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot.armMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void waitFor (double time)
    {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {

        }
        return;
    }

    public void moveCarousel (double time, double power)
    {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {
            robot.rightSpinnyWheel.setPower(power);
            robot.leftSpinnyWheel.setPower(power);
        }
        robot.leftSpinnyWheel.setPower(0);
        robot.rightSpinnyWheel.setPower(0);
    }

    public static void moveArmv2(SpaghettiHardware robot, int armUp)
    {
        int rightTargetPos = robot.armMotorTwo.getTargetPosition();
        int leftTargetPos = robot.armMotorOne.getTargetPosition();
        int offset = Math.abs(robot.armMotorTwo.getCurrentPosition() - robot.armMotorTwo.getTargetPosition());
        currentPosition = robot.armMotorTwo.getCurrentPosition();
        leftArmOffset = currentPosition - robot.armMotorOne.getCurrentPosition();

        if (armUp == 1 && armTime.time() >= 40)
        {
            robot.armMotorTwo.setTargetPosition(170);
            robot.armMotorOne.setTargetPosition(170);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() + 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() + 10);

            robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armMotorOne.setPower(0.6);
            robot.armMotorTwo.setPower(0.6);

            armTime.reset();
        }

        else if (offset >= 5 && armTime.time() >= 500)
        {
            robot.armMotorOne.setTargetPosition(currentPosition + leftArmOffset);
            robot.armMotorTwo.setTargetPosition(currentPosition);
            // robot.leftArm.setTargetPosition(currentPosition + leftArmOffset);

            robot.armMotorTwo.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.armMotorOne.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.armMotorTwo.setPower(1);
            robot.armMotorOne.setPower(1);


        }
    }
}
