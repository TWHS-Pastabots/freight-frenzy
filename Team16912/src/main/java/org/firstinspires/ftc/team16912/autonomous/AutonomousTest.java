package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LinguineAutonomousV2")
public class AutonomousTest extends LinearOpMode {


    LinguineHardware robot = new LinguineHardware();
    SampleMecanumDrive drive;

    ElapsedTime runTime = new ElapsedTime();

    Trajectory toShipment, toCarousel, toFinish;

    Pose2d startPose;

    private String alliance = "blue", side = "left";
    boolean isWaiting = false;

    OpenCvInternalCamera webcam;
    BarcodePipelineTest pipeline;




    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BarcodePipelineTest();
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        BarcodePipelineTest.ObjectPosition pos = pipeline.getAnalysis();


        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        PoseStorage.initPoses();

        config();
        drive.setPoseEstimate(startPose);


        initTrajectories();

        closeClaw();

        waitForStart();

        if (isStarted()) {

            if (isWaiting) wait5();

            // No camera yet so robot will always deliver to top for now
            deliverShipment(barcodeToInt(pos));

            robot.cSpinner.setDirection(DcMotorEx.Direction.REVERSE);
            spinCarousel();

            //slow down spinner
            robot.cSpinner.setVelocity(300);

            //wait 1.5 seconds for the duck to fall
            sleep(2500);
            robot.cSpinner.setVelocity(0);

            runArmToStart();

            setToFinish();
        }

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

    private int barcodeToInt(BarcodePipelineTest.ObjectPosition position) {

        switch (position) {
            case LEFT:
                return 1;

            case CENTER:
                return 2;

            case RIGHT:
                return 3;
        }

        return 0;
    }

    private void deliverShipment(int pos) {

        int encoderPos;

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

            default: encoderPos = 0;

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
        while (robot.armEncoder.getCurrentPosition() < -1000) {
            for (DcMotorEx motor : robot.motorArms) {
                motor.setPower(.5);
            }
        }
        for (DcMotorEx motor : robot.motorArms) motor.setPower(0);
    }

    private void wait5() { sleep (10000); }

    // Alliance configuration
    private void config() {

        // Alliance Config
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

            telemetry.addData("Start Position: ", alliance.toUpperCase() + " " + side.toUpperCase());
            telemetry.update();
        }

        telemetry.clearAll();
        telemetry.addLine("ALLIANCE SELECTION CONFIRMED");
        telemetry.update();

        sleep(1000);
        telemetry.clear();
        telemetry.addLine("Select wait time");
        telemetry.update();

        sleep(1000);
        telemetry.clear();
        telemetry.update();

        // Add Wait5
        while (!gamepad1.left_bumper) {
            if (gamepad1.cross && !isWaiting) {
                isWaiting = true;
                telemetry.addLine("Waiting 5 seconds on start");
            }

            else if (gamepad1.cross && isWaiting) {
                isWaiting = false;
                telemetry.addLine("No wait on start");
            }

            telemetry.update();

        }

        telemetry.clearAll();
        telemetry.addLine("CONFIRMED. READY TO START");
        telemetry.update();
    }

}
