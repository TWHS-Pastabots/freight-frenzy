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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LinguineAutonomousV1")
public class Autonomous extends LinearOpMode {


    LinguineHardware robot = new LinguineHardware();
    SampleMecanumDrive drive;

    ElapsedTime runTime = new ElapsedTime();

    Trajectory start, toShipment, moveBack, toCarousel, toWarehouse, toFinish, toSetup, redBlocks, blueBlocks;

    Pose2d startPose;

    private String alliance = "BLUE", side = "LEFT";
    private int waitTime = 0;

    private boolean goToCarousel;


    OpenCvInternalCamera webcam;
    BarcodePipeline pipeline;

    private final int armStartPosition = -1000;


    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BarcodePipeline();
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "*Camera could not be opened*");
                telemetry.update();
            }
        });


        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        PoseStorage.initPoses();


        while (!gamepad1.triangle) {
            telemetry.addData("Left Brightness: ", pipeline.R1Y);
            telemetry.addData("Middle Brightness: ", pipeline.R2Y);
            telemetry.addData("Right Brightness: ", pipeline.R3Y);
            telemetry.addData("Deliver To", pipeline.getAnalysis());
            telemetry.update();
        }

        config();
        drive.setPoseEstimate(startPose);

        goToCarousel = (alliance.equals("RED") && side.equals("LEFT"))
                            || (alliance.equals("BLUE") && side.equals("RIGHT"));


        initTrajectories();

        closeClaw();

        waitForStart();

        if (isStarted()) {

            sleep(waitTime);

            drive.followTrajectory(start);

            deliverShipment();

            if (goToCarousel) {
                drive.followTrajectory(toCarousel);
                runArmTo(armStartPosition);
                spinCarousel();
            }

            closeClaw();
            goToFinish();
            //pickupBlock(1);

        }

    }


    private void initTrajectories() {


        start = drive.trajectoryBuilder(startPose)
                .forward(12)
                .build();

        switch(alliance) {

            case ("RED"): {
                toShipment = drive.trajectoryBuilder(start.end())
                        .lineToConstantHeading(PoseStorage.RedHub)
                        .build();
                moveBack = drive.trajectoryBuilder(toShipment.end())
                        .forward(-12)
                        .build();


                switch (side) {

                    case ("LEFT"): {
                        toCarousel = drive.trajectoryBuilder(moveBack.end())
                                .lineToConstantHeading(PoseStorage.RedCarousel)
                                .build();
                        toFinish = drive.trajectoryBuilder(toCarousel.end())
                                .lineToLinearHeading(PoseStorage.RedStorageUnit)
                                .build();
                        break;
                    }

                    case ("RIGHT"): {

                        toSetup = drive.trajectoryBuilder(moveBack.end())
                                .lineToLinearHeading(PoseStorage.RedWarehouseSetup)
                                .addTemporalMarker(.25, this::runArmToStart)
                                .build();

                        toWarehouse = drive.trajectoryBuilder(toSetup.end())
                                .strafeRight(50)
                                .build();

                        redBlocks = drive.trajectoryBuilder(toWarehouse.end())
                                .splineToSplineHeading(PoseStorage.RedWarehouseBlocks, -145.0)
                                .build();
                        break;
                    }
                }


                break;
            }

            case ("BLUE"): {

                toShipment = drive.trajectoryBuilder(start.end())
                        .lineToLinearHeading(PoseStorage.BlueHub)
                        .build();

                moveBack = drive.trajectoryBuilder(toShipment.end())
                        .forward(-9)
                        .build();

                switch (side) {

                    case ("LEFT"): {


                        toSetup = drive.trajectoryBuilder(moveBack.end())
                                .lineToLinearHeading(PoseStorage.BlueWarehouseSetup)
                                .addTemporalMarker(.25, this::runArmToStart)
                                .build();

                        toWarehouse = drive.trajectoryBuilder(toSetup.end())
                                .strafeLeft(40)
                                .build();

                        break;
                    }

                    case ("RIGHT"): {
                        toCarousel = drive.trajectoryBuilder(moveBack.end())
                                .lineToLinearHeading(PoseStorage.BlueCarousel)
                                .build();

                        toFinish = drive.trajectoryBuilder(toCarousel.end())
                                .lineToLinearHeading(PoseStorage.BlueStorageUnit)
                                .build();

                        break;
                    }
                }

                break;
            }
        }
    }


    private void deliverShipment()
    {
        int encoderPos = pipeline.getAnalysis();

        telemetry.addData("Delivering to: ", encoderPos);
        telemetry.update();

        // Drive to position
        drive.followTrajectory(toShipment);

        // Run arm to deliver position
        runArmTo(encoderPos);
        // Move forwards based on which level

        drive.followTrajectory(moveBack);

        // Claw actions
        openClaw();
        sleep(100);

        double initTime = runTime.seconds();

        while (runTime.seconds() - initTime < 1) openClaw();
        sleep(50);
        initTime = runTime.seconds();
        while (runTime.seconds() - initTime < 1) closeClaw();
        sleep(50);
    }

    private void spinCarousel() {



        switch (alliance) {

            case "BLUE": {
                robot.cSpinner.setDirection(DcMotorEx.Direction.REVERSE);
                break;
            }

            case "RED": {

                robot.cSpinner.setDirection(DcMotorEx.Direction.FORWARD);
                break;
            }

        }

        // set spinner speed
        robot.cSpinner.setVelocity(200);

        // wait 2.5 seconds for the duck to fall
        sleep(3000);
        robot.cSpinner.setVelocity(0);

    }

    private void goToFinish() {
        if (goToCarousel) drive.followTrajectory(toFinish);
        else {
            drive.followTrajectory(toSetup);
            drive.followTrajectory(toWarehouse);
        }
    }

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

    private void runArmTo(int targetPos)
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


    // Alliance configuration
    private void config()
    {
        String[] alliances = new String[]{"RED", "BLUE"};
        String[] sides = new String[]{"LEFT", "RIGHT"};
        int allianceIndex = 0;
        int sideIndex = 0;

        double LastClick = runTime.milliseconds();

        // Alliance Config
        while (!gamepad1.cross)
        {
            if(runTime.milliseconds() - LastClick < 150) continue;

            if(gamepad1.dpad_right)
            {
                sideIndex++;
                if(sideIndex >= sides.length)
                {
                    sideIndex = 0;
                    allianceIndex++;
                }
            }
            else if (gamepad1.dpad_left)
            {
                sideIndex--;
                if(sideIndex < 0)
                {
                    sideIndex = sides.length-1;
                    allianceIndex--;
                }
                if(allianceIndex < 0)
                {
                    allianceIndex = alliances.length-1;
                }
            }

            allianceIndex %= alliances.length;
            alliance = alliances[allianceIndex];
            side = sides[sideIndex];
            startPose = PoseStorage.DetStartPose(allianceIndex, sideIndex);


//            if (gamepad1.circle) {
//                alliance = "red";
//                side = "right";
//                startPose = PoseStorage.RedRight;
//            }
//
//            else if (gamepad1.square) {
//                alliance = "red";
//                side = "left";
//                startPose = PoseStorage.RedLeft;
//            }
//
//            else if (gamepad1.triangle) {
//                alliance = "blue";
//                side = "right";
//                startPose = PoseStorage.BlueRight;
//            }
//
//            else if (gamepad1.cross) {
//                alliance = "blue";
//                side = "left";
//                startPose = PoseStorage.BlueLeft;
//            }

            telemetry.addData("Start Position: ", alliance + " " + side + "\nPress X to confirm");
            telemetry.update();
            LastClick = runTime.milliseconds();
        }

        telemetry.clearAll();
        telemetry.addLine("ALLIANCE SELECTION CONFIRMED");
        telemetry.update();
        sleep(500);
        telemetry.clear();

//        sleep(1000);
//        telemetry.clear();
//        telemetry.update();

        // Select wait time
        while (!gamepad1.cross)
        {
            if(runTime.milliseconds() - LastClick < 200) continue;

            if (gamepad1.dpad_up)
            {
                waitTime += 1000;
            }

            else if (gamepad1.dpad_down)
            {
                if(waitTime > 0) waitTime -= 1000;
            }

            telemetry.addLine("Waiting " + waitTime/1000 + " seconds on start" + "\nPress X to start");
            telemetry.update();
            LastClick = runTime.milliseconds();
        }

        sleep(500);
        telemetry.clearAll();
        telemetry.addLine("CONFIRMED. READY TO START");
        telemetry.update();
    }

    private void pickupBlock(int count) {
        if (count != 0) {
            for (int i = 0; i < count; i++) {
                if(alliance.equals("RED")) {

                    drive.followTrajectory(redBlocks);

                    runArmTo(250);
                    openClaw();

                    closeClaw();
                    sleep(500);
                    runArmTo(-1000);

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToLinearHeading(toWarehouse.end(), 100.0)
                            .build()
                    );

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeLeft(55)
                            .build()
                    );

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY() + 14, drive.getPoseEstimate().getHeading() + Math.PI))
                            .build()
                    );

                    runArmTo(-2590);
                }

                else
                {
                    drive.followTrajectory(
                            drive.trajectoryBuilder(toWarehouse.end())
                                    .forward(10)
                                    .build()
                    );

                    drive.turn(Math.toRadians(105));

                    runArmTo(250);
                    openClaw();

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .forward(7)
                                    .build()
                    );

                    closeClaw();
                    sleep(500);
                    runArmTo(-1000);

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .forward(-7)
                                    .build()
                    );

                    drive.turn(Math.toRadians(-105));

                    drive.followTrajectory(
                            drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .forward(-10)
                                    .build()
                    );
                }


            }
        }

    }
}
