package org.firstinspires.ftc.team16909.autonomousv2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;

public class Actions {
    private FettuccineHardware robot;

    boolean grabHold = false;
    boolean justMoved = false;
    ElapsedTime armTime = null;
    ElapsedTime lockoutTime = null;
    ElapsedTime timeSinceMove = null;

    // Arm Values
    int currentPosition = 0;
    int leftArmOffset = 0;



    public Actions(FettuccineHardware robot)
    {
        this.robot = robot;

        armTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeSinceMove = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        lockoutTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void waitFor (double time) {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {

        }
        return;
    }

    public void moveCarousel (double time, double power) {

        ElapsedTime moveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double startTime = moveTime.seconds();

        while (moveTime.seconds() - startTime < time) {
            robot.carousel.setPower(power);
        }
        robot.carousel.setPower(0);
    }

    public void moveArm (int position, int trim)
    {

        int rightTargetPos = robot.rightArm.getTargetPosition();
        int leftTargetPos = robot.leftArm.getTargetPosition();
        int offset = Math.abs(robot.rightArm.getCurrentPosition() - robot.rightArm.getTargetPosition());
        currentPosition = robot.rightArm.getCurrentPosition();
        leftArmOffset = currentPosition - robot.leftArm.getCurrentPosition();

        if (position == 3 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(140 + trim);
            robot.leftArm.setTargetPosition(140 + trim);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() + 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() + 10);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (position == 2 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(73 + trim);
            robot.leftArm.setTargetPosition(73 + trim);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() - 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() - 10);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (position == 1 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(15 + trim);
            robot.leftArm.setTargetPosition(15 + trim);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() - 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() - 10);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (position == 0 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(0 + trim);
            robot.leftArm.setTargetPosition(0 + trim);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() - 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() - 10);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();
        }
        else if (offset >= 5 && armTime.time() >= 500)
        {
            robot.leftArm.setTargetPosition(currentPosition + leftArmOffset);
            robot.rightArm.setTargetPosition(currentPosition);
            // robot.leftArm.setTargetPosition(currentPosition + leftArmOffset);

            robot.rightArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.leftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.rightArm.setPower(1);
            robot.leftArm.setPower(1);

            justMoved = false;
        }

        /*if (currentPosition == lastPosition)
        {
            if (!canRun)
            {
                lockoutTime.reset();
            }
            canRun = true;
        }
        else
        {
            lastPosition = currentPosition;
            canRun = false;
        }*/

//        telemetry.addData("rF", robot.rightFront.getVelocity());
//        telemetry.addData("lF", robot.leftFront.getVelocity());
//        telemetry.addData("rR", robot.rightRear.getVelocity());
//        telemetry.addData("lR", robot.leftRear.getVelocity());
//        telemetry.addData("Arm One Pos", robot.rightArm.getCurrentPosition());
//        telemetry.addData("Arm One Target", robot.rightArm.getTargetPosition());
//        telemetry.addData("Arm Two Pos", robot.leftArm.getCurrentPosition());
//        telemetry.addData("Arm Two Target", robot.leftArm.getTargetPosition());
//        telemetry.addData("Arm Time", armTime.time());
//        telemetry.addData("Just Moved", justMoved);
//        telemetry.update();
    }
}
