package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16912.teleop.Linguine;
import org.firstinspires.ftc.team16912.util.LinguineHardware;

import java.util.ArrayList;
import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LinguineAutonomousV1")
public class Autonomous extends LinearOpMode {


    LinguineHardware robot = new LinguineHardware();
    SampleMecanumDrive drive;

    ElapsedTime runTime = new ElapsedTime();

    Trajectory toShipment, toCarousel, toFinish;

    Pose2d startPose;

    private String alliance = "blue", side = "left";

    public void runOpMode() {

        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        PoseStorage.initPoses();

        config();
        drive.setPoseEstimate(startPose);


        initTrajectories();

        closeClaw();

        waitForStart();

        // No camera yet so robot will always deliver to top for now
        deliverShipment(1);

        robot.cSpinner.setDirection(DcMotorEx.Direction.REVERSE);
        spinCarousel();

        //slow down spinner
        robot.cSpinner.setVelocity(300);

        //wait 1.5 seconds for the duck to fall
        sleep(2500);
        robot.cSpinner.setVelocity(0);

        setToFinish();
        runArmToStart();
    }


    private void initTrajectories() {


        switch(alliance) {

            case ("red"): {

                toShipment = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(PoseStorage.RedHub)
                        .build();

                toCarousel = drive.trajectoryBuilder(toShipment.end())
                        .lineToLinearHeading(PoseStorage.RedCarousel)
                        .build();

                toFinish = drive.trajectoryBuilder(toCarousel.end())
                        .lineToLinearHeading(PoseStorage.RedFinish)
                        .build();

                break;
            }

            case ("blue"): {

                toShipment = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(PoseStorage.BlueHub)
                        .build();

                toCarousel = drive.trajectoryBuilder(toShipment.end())
                        .lineToLinearHeading(PoseStorage.BlueCarousel)
                        .build();

                toFinish = drive.trajectoryBuilder(toCarousel.end())
                        .lineToLinearHeading(PoseStorage.BlueFinish)
                        .build();

                break;
            }
        }









    }

    private void deliverShipment(int pos) {

        int encoderPos = 0;

        switch (pos) {

            case 1: {
                encoderPos = -3000;
                break;
            }

            case 2: {
                encoderPos = -4248;
                break;
            }

            case 3: {
                encoderPos = -4748;
                break;
            }

        }

        // Drive to position
        drive.followTrajectory(toShipment);

        // Run arm to deliver position
        while (robot.armEncoder.getCurrentPosition() > encoderPos) {
            for (DcMotorEx motor : robot.motorArms) motor.setPower(-.5);
            telemetry.addData("Arm Position: ", robot.armEncoder.getCurrentPosition());
            telemetry.update();
        }

        for (DcMotorEx motor : robot.motorArms) motor.setPower(0);

        // Claw actions
        openClaw();
        sleep(100);
        closeClaw();

        double initTime = runTime.seconds();

        while (runTime.seconds() - initTime < 1) openClaw();
        sleep(50);
        initTime = runTime.seconds();
        while (runTime.seconds() - initTime < 1) closeClaw();
        sleep(50);


        // Run arm to start position
        while (robot.armEncoder.getCurrentPosition() > 0)
            for (DcMotorEx motor : robot.motorArms) motor.setPower(-.5);
    }

    private void spinCarousel() {

        drive.followTrajectory(toCarousel);

        switch (alliance) {

            case "blue": {
                robot.cSpinner.setDirection(DcMotorEx.Direction.FORWARD);
                break;
            }

            case "red": {
                robot.cSpinner.setDirection(DcMotorEx.Direction.REVERSE);
                break;
            }

        }

        robot.cSpinner.setVelocity(3000);

    }

    private void setToFinish() { drive.followTrajectory(toFinish); }

    // Closes claw
    private void closeClaw() { robot.servoClaw.setPosition(.7); }

    // Opens claw
    private void openClaw() { robot.servoClaw.setPosition(-1); }

    // Return arm to start
    private void runArmToStart() {
        while (robot.armEncoder.getCurrentPosition() < -1000)
            for (DcMotorEx motor : robot.motorArms) motor.setPower(.5);
    }

    // Alliance configuration
    private void config() {

        while (!gamepad1.right_bumper) {

            if (gamepad1.circle) {
                alliance = "red";
                side = "right";
                startPose = PoseStorage.RedRight;
            }

            else if (gamepad1.square) {
                alliance = "red";
                side = "left";
                startPose = PoseStorage.RedLeft;
            }

            else if (gamepad1.triangle) {
                alliance = "blue";
                side = "right";
                startPose = PoseStorage.BlueRight;
            }

            else if (gamepad1.cross) {
                alliance = "blue";
                side = "left";
                startPose = PoseStorage.BlueLeft;
            }

            telemetry.addData("Start Position: ", alliance + " " + side);
            telemetry.update();
        }
    }

}
