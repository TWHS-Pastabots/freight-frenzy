package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team16912.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16912.util.LinguineHardware;
import org.firstinspires.ftc.team16912.util.Util;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "LinguineAutonomousV2")
public class AutonomousV2 extends LinearOpMode {


    LinguineHardware robot = new LinguineHardware();
    SampleMecanumDrive drive;

    ElapsedTime runTime = new ElapsedTime();

    Trajectory toShipment, toWarehouseSetup, toWarehouse, toCarousel, toBlockPickup;

    Pose2d startPose;

    private String alliance = "BLUE", side = "LEFT";
    private int waitTime = 0;
    private int blockToPickUp = 0;


    OpenCvInternalCamera webcam;
    BarcodePipeline pipeline;


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
        Util.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        PoseStorage.initPoses();


        while (!gamepad1.cross) {
            telemetry.addData("Left Brightness: ", pipeline.R1Y);
            telemetry.addData("Middle Brightness: ", pipeline.R2Y);
            telemetry.addData("Right Brightness: ", pipeline.R3Y);
            telemetry.addData("Deliver To", pipeline.getAnalysis());
            telemetry.update();
        }

        config();

        drive.setPoseEstimate(startPose);

        initTrajectories();

        // Closes claw to grip freight at start
        Util.closeClaw(.01);

        waitForStart();

        if (isStarted()) {

            sleep(waitTime);
            // Scans barcode and delivers freight to correct level
            deliverShipment();
            // Goes to either warehouse or spins carousel
            gotoFinish();
        }
    }

    public void PickUpBlock()
    {
        Util.closeClaw(.5);
        Util.runArmTo(-1000);
    }

    public void gotoFinish()
    {
        switch(alliance) {

            case ("RED"): {


                switch (side) {

                    case ("LEFT"): {

                        drive.followTrajectory(toCarousel);
                        Util.setSpinnerDirection('r');
                        spinCarousel();
                        break;
                    }

                    case ("RIGHT"):
                    {
                        if(blockToPickUp == 0)
                        {
                            drive.followTrajectory(toWarehouseSetup);
                            drive.followTrajectory(toWarehouse);
                        }

                        // Pick up blocks amount of times set in config
                        for(int i = 0; i < blockToPickUp; i++)
                        {
                            drive.followTrajectory(toWarehouseSetup);
                            drive.followTrajectory(toWarehouse);
                            drive.followTrajectory(toBlockPickup);
                            PickUpBlock();
                        }

                        break;
                    }
                }


                break;
            }

            case ("BLUE"): {



                switch (side) {

                    case ("LEFT"): {

                        break;
                    }

                    case ("RIGHT"): {
                        int i = 0;

                        break;
                    }
                }

                break;
            }
        }
    }


    private void initTrajectories() {

        switch(alliance) {

            case ("RED"): {
                toShipment = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(PoseStorage.RedHub)
                        .addTemporalMarker(.1, ()-> {
                            Util.runArmTo(pipeline.getAnalysis());
                        })
                        .build();

                switch (side) {

                    case ("LEFT"): {

                        toCarousel = drive.trajectoryBuilder(toShipment.end())
                                .splineToConstantHeading(PoseStorage.RedCarousel, Math.toRadians(190))
                                .build();
                        break;
                    }

                    case ("RIGHT"): {

                        toWarehouseSetup = drive.trajectoryBuilder(toShipment.end())
                                .lineToLinearHeading(PoseStorage.RedWarehouseSetup)
                                .addTemporalMarker(.25, ()-> {
                                    Util.runArmTo(0);
                                })
                                .build();

                        toWarehouse = drive.trajectoryBuilder(toWarehouseSetup.end())
                                .strafeRight(40)
                                .build();

                        toBlockPickup = drive.trajectoryBuilder(toWarehouse.end())
                                .lineToLinearHeading(PoseStorage.RedWarehouseBlocks)
                                .addTemporalMarker(.01, ()->{
                                    Util.runArmTo(250);
                                })
                                .build();
                        break;
                    }
                }


                break;
            }

            case ("BLUE"): {

                toShipment = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(PoseStorage.BlueHub)
                        .build();

                switch (side) {

                    case ("LEFT"): {

                        break;
                    }

                    case ("RIGHT"): {
                        int i = 0;

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

        Util.runArmTo(encoderPos);
        // Drive to position
        drive.followTrajectory(toShipment);

        // Claw actions
        Util.openClaw(.5);
    }

    private void spinCarousel()
    {
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
        robot.cSpinner.setVelocity(250);

        // wait 2.5 seconds for the duck to fall
        sleep(1800);
        robot.cSpinner.setVelocity(0);

    }


    // Alliance configuration
    private void config()
    {
        String[] alliances = new String[]{"RED", "BLUE"};
        String[] sides = new String[]{"LEFT", "RIGHT"};
        int allianceIndex = 0;
        int sideIndex = 0;

        double LastClick = runTime.milliseconds();
        sleep(400);

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

            telemetry.addData("Start Position: ", alliance + " " + side + "\nPress X to confirm");
            telemetry.update();
            LastClick = runTime.milliseconds();
        }

        telemetry.clearAll();
        telemetry.addLine("ALLIANCE SELECTION CONFIRMED");
        telemetry.update();
        sleep(500);
        telemetry.clear();


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

        telemetry.clearAll();
        telemetry.addLine("Wait Time Confirmed");
        telemetry.update();
        sleep(500);
        telemetry.clear();

        while (!gamepad1.cross)
        {
            if(runTime.milliseconds() - LastClick < 200) continue;

            if (gamepad1.dpad_up)
            {
                blockToPickUp++;
            }

            else if (gamepad1.dpad_down)
            {
                if(blockToPickUp > 0) blockToPickUp--;
            }

            telemetry.addLine("Blocks To Pick Up " + blockToPickUp + "\nPress X to start");
            telemetry.update();
            LastClick = runTime.milliseconds();
        }

        sleep(500);
        telemetry.clearAll();
        telemetry.addLine("CONFIRMED. READY TO START");
        telemetry.update();
    }
}
