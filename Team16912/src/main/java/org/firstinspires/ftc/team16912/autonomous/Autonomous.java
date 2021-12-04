package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16912.util.LinguineHardware;

import java.util.ArrayList;
import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LinguineAutonomousV1")
public class Autonomous extends LinearOpMode {

    private List<Trajectory> trajectories = new ArrayList<Trajectory>();


    LinguineHardware robot = new LinguineHardware();
    SampleMecanumDrive drive;

    Trajectory toShipment, toCarousel, toFinish;

    Pose2d startPose;

    private String alliance = "", side = "";

    public void runOpMode() {

        PoseStorage.initPoses();

        config();

        initTrajectories();

        // No camera yet so robot will always deliver to top for now
        deliverShipment(3);

        spinCarousel();

        setToFinish();
    }


    private void initTrajectories() {

        toShipment = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(PoseStorage.shipmentHub)
                .build();

        toCarousel = drive.trajectoryBuilder(toShipment.end())
                .lineToLinearHeading(PoseStorage.carousel)
                .build();

        toFinish = drive.trajectoryBuilder(toCarousel.end())
                .lineToLinearHeading(PoseStorage.finish)
                .build();

    }

    private void deliverShipment(int pos) {

        int encoderPos = 0;

        switch (pos) {

            case 1: encoderPos = 6000;

            case 2: encoderPos = 4000;

            case 3: encoderPos = 2000;

        }

        // Drive to position
        drive.followTrajectory(toShipment);

        // Run arm to deliver position
        while (robot.armEncoder.getCurrentPosition() < encoderPos)
            for (DcMotorEx motor : robot.motorArms) motor.setPower(.5);

        // Claw actions
        openClaw();
        sleep(500);
        closeClaw();

        // Run arm to start position
        while (robot.armEncoder.getCurrentPosition() > 0)
            for (DcMotorEx motor : robot.motorArms) motor.setPower(-.5);
    }

    private void spinCarousel() {

        drive.followTrajectory(toCarousel);

        switch (alliance) {

            case "blue": robot.cSpinner.setDirection(DcMotorEx.Direction.FORWARD);

            case "red": robot.cSpinner.setDirection(DcMotorEx.Direction.REVERSE);

        }

        robot.cSpinner.setVelocity(3000);

    }

    private void setToFinish() { drive.followTrajectory(toFinish); }

    // Closes claw
    private void closeClaw() { robot.servoClaw.setPosition(.7); }

    // Opens claw
    private void openClaw() { robot.servoClaw.setPosition(-1); }


    // Alliance configuration
    private void config() {


        if (gamepad1.circle) {
            alliance = "blue";
            telemetry.addLine("Alliance: " + alliance);
            telemetry.update();
        }
        else if (gamepad1.square) {
            alliance = "red";
            telemetry.addLine("Alliance" + alliance);
            telemetry.update();
        }

        switch (alliance) {
            case "blue": {
                if (gamepad1.dpad_left) {
                    side = "left";
                    startPose = PoseStorage.blueLeft;
                    telemetry.addLine("Side: " + side);
                    telemetry.update();
                }
                else if (gamepad1.dpad_right) {
                    side = "right";
                    startPose = PoseStorage.blueRight;
                    telemetry.addLine("Side: " + side);
                    telemetry.update();
                }
            }

            case "red": {
                if (gamepad1.dpad_left) {
                    side = "left";
                    startPose = PoseStorage.redLeft;
                    telemetry.addLine("Side: " + side);
                    telemetry.update();
                }
                else if (gamepad1.dpad_right) {
                    side = "right";
                    startPose = PoseStorage.redRight;
                    telemetry.addLine("Side: " + side);
                    telemetry.update();
                }
            }
        }

    }
}
