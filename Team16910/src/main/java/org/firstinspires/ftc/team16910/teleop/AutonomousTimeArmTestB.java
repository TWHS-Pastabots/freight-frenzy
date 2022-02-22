package org.firstinspires.ftc.team16910.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;


public class AutonomousTimeArmTestB extends LinearOpMode {


    SpaghettiHardware robot = new SpaghettiHardware();
    ElapsedTime runTime = new ElapsedTime();

    private double initTime;

    public void runOpMode() {

        robot.init(hardwareMap);


        waitForStart();

        initTime = runTime.seconds();

        while (runTime.seconds() - initTime < 1.8) close();
        resetInit();

        while (runTime.seconds() - initTime < .1) stay();
        resetInit();

        while (runTime.seconds() - initTime < .75) runForwards();
        resetInit();

        while (runTime.seconds() - initTime < .1) stay();
        resetInit();

        while (runTime.seconds() - initTime < 1.5) runLeft();
        resetInit();

        while (runTime.seconds() - initTime < .1) stay();
        resetInit();

        while (runTime.seconds() - initTime < 1.3) moveDown();
        resetInit();

        while (runTime.seconds() - initTime < .1) hold();
        resetInit();

        while (runTime.seconds() - initTime < .1) stay();
        resetInit();

        while (runTime.seconds() - initTime < .5) open();
        resetInit();

        while (runTime.seconds() - initTime < .1) stay();
        resetInit();

        while (runTime.seconds() - initTime < 1.5) moveUp();
        resetInit();

        while (runTime.seconds() - initTime < .5) close();
        resetInit();

        while (runTime.seconds() - initTime < .1) hold();
        resetInit();

        while (runTime.seconds() - initTime < .1) stay();
        resetInit();

        while (runTime.seconds() - initTime < .8) turnRight();
        resetInit();

        while (runTime.seconds() - initTime < .1) stay();
        resetInit();

        while (runTime.seconds() - initTime < 1.8) runForwardsFast();
        resetInit();

        stop();
    }

    private void resetInit()
    {
        initTime = runTime.seconds();
    }

    private void turnRight()
    {
        robot.leftFront.setPower(-.5);
        robot.leftRear.setPower(-.5);
        robot.rightRear.setPower(.5);
        robot.rightFront.setPower(.5);
    }

    private void turnLeft()
    {
        robot.leftFront.setPower(.5);
        robot.leftRear.setPower(.5);
        robot.rightRear.setPower(-.5);
        robot.rightFront.setPower(-.5);
    }

    private void runForwards()
    {
        robot.leftFront.setPower(-.5);
        robot.leftRear.setPower(-.5);
        robot.rightRear.setPower(-.5);
        robot.rightFront.setPower(-.5);
    }

    private void runBackwards()
    {
        robot.leftFront.setPower(-.5);
        robot.leftRear.setPower(-.5);
        robot.rightRear.setPower(-.5);
        robot.rightFront.setPower(-.5);
    }

    private void runRight()
    {
        robot.leftFront.setPower(.5);
        robot.leftRear.setPower(-.5);
        robot.rightRear.setPower(.5);
        robot.rightFront.setPower(-.5);
    }

    private void runLeft()
    {
        robot.leftFront.setPower(-.5);
        robot.leftRear.setPower(.5);
        robot.rightRear.setPower(-.5);
        robot.rightFront.setPower(.5);
    }

    private void spinCarousel()
    {
        robot.spinnyWheel.setPower(0.4);
    }

    private void stay()
    {
        robot.leftFront.setPower(0.0);
        robot.leftRear.setPower(0.0);
        robot.rightRear.setPower(0.0);
        robot.rightFront.setPower(0.0);
    }

    private void moveDown()
    {
        robot.armMotorOne.setPower(-.6);
        robot.armMotorTwo.setPower(-.6);
    }

    private void hold()
    {
        robot.armMotorOne.setPower(0.0);
        robot.armMotorTwo.setPower(0.0);
    }

    private void moveUp()
    {
        robot.armMotorOne.setPower(.3);
        robot.armMotorTwo.setPower(.3);
    }

    private  void open()
    {
        robot.grabberServo.setPosition(-1);
    }

    private void close()
    {
        robot.grabberServo.setPosition(1);
    }

    private void runForwardsFast()
    {
        robot.leftFront.setPower(-1.0);
        robot.leftRear.setPower(-1.0);
        robot.rightRear.setPower(-1.0);
        robot.rightFront.setPower(-1.0);
    }
}