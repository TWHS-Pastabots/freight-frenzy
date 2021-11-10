package org.firstinspires.ftc.team16912.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16912.util.LinguineHardware;

@Autonomous(name = "Forwards Test")
public class TimeForwards extends LinearOpMode {


    LinguineHardware robot = new LinguineHardware();
    ElapsedTime runTime = new ElapsedTime();

    private double initTime;

    public void runOpMode() {

        robot.init(hardwareMap, true);


        waitForStart();

        initTime = runTime.seconds();

        for (int i = 0; i < 4; i++) {
            while (runTime.seconds() - initTime < 1.0) runForwards();
            resetInit();

            while (runTime.seconds() - initTime < 1.0) turnRight();
            resetInit();
        }


        stop();
    }

    private void resetInit() {
        initTime = runTime.seconds();
    }

    private void turnRight() {
        robot.motorLeftFront.setPower(-.5);
        robot.motorLeftRear.setPower(-.5);
        robot.motorRightRear.setPower(.5);
        robot.motorRightFront.setPower(.5);
    }

    private void turnLeft() {
        robot.motorLeftFront.setPower(.5);
        robot.motorLeftRear.setPower(.5);
        robot.motorRightRear.setPower(-.5);
        robot.motorRightFront.setPower(-.5);
    }

    private void runForwards() {
        for (DcMotorEx motor : robot.wheels) motor.setPower(.5);
    }

    private void runBackwards() {
        for (DcMotorEx motor : robot.wheels) motor.setPower(-.5);
    }
}
