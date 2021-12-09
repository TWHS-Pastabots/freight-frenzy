package org.firstinspires.ftc.team16910.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;

@TeleOp(name = "Autonomous")
public class AutonomousTime extends LinearOpMode {


    SpaghettiHardware robot = new SpaghettiHardware();
    ElapsedTime runTime = new ElapsedTime();

    private double initTime;

    public void runOpMode() {

        robot.init(hardwareMap);


        waitForStart();

        initTime = runTime.seconds();

            while (runTime.seconds() - initTime < 1.5) runRight();
            resetInit();

            while (runTime.seconds() - initTime < .25) turnLeft();
            resetInit();

            while (runTime.seconds() - initTime < .25) turnLeft();
            resetInit();

            while (runTime.seconds() - initTime < 2.0) spinCarousel();
            resetInit();

            while (runTime.seconds() - initTime < 1.0) turnRight();
            resetInit();

            while (runTime.seconds() - initTime < .5625) runLeft();
            resetInit();

            while (runTime.seconds() - initTime < 7.0) runForwards();
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
        robot.leftFront.setPower(.5);
        robot.leftRear.setPower(.5);
        robot.rightRear.setPower(.5);
        robot.rightFront.setPower(.5);
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
        robot.spinnyWheel.setPower(.6);
    }

    private void stay()
    {
        robot.leftFront.setPower(0.0);
        robot.leftRear.setPower(0.0);
        robot.rightRear.setPower(0.0);
        robot.rightFront.setPower(0.0);
    }
}