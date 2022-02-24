package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "I'm So Done With This")
public class DesperateBackup extends LinearOpMode {


    SpaghettiHardware robot = new SpaghettiHardware();
    SampleMecanumDrive drive;

    ElapsedTime runTime = new ElapsedTime();

    Trajectory toShipment, toWarehouseSetup, toWarehouse, toCarousel;

    Pose2d startPose;

    private String alliance = "BLUE", side = "LEFT";
    private int waitTime = 0;


    OpenCvInternalCamera webcam;
    BarcodeReader reader;

    public static Pose2d RedLeft, RedRight, BlueLeft, BlueRight;

    public static Pose2d BlueWarehouseBarcode,BlueWarehouseHub_3, BlueWarehouseHub, BlueWarehouseEnd_3, BlueWarehouseEnd;
    public static Pose2d BlueStorageBarcode,BlueStorageCarousel, BlueStorageHub_3, BlueStorageHubApproach, BlueStorageHub, BlueStorageEnd;

    public static Pose2d RedWarehouseBarcode,RedWarehouseHub_3, RedWarehouseHub, RedWarehouseEnd_3, RedWarehouseEnd;
    public static Pose2d RedStorageBarcode, RedStorageCarousel, RedStorageHub_3, RedStorageHubApproach, RedStorageHub, RedStorageEnd_3, RedStorageEnd;

    public static void Positions()
    {
        RedLeft = new Pose2d(-36.04860248750233, -60, Math.toRadians(-90));
        RedRight = new Pose2d(12, -60, Math.toRadians(-90));
        BlueLeft = new Pose2d(13.04860248750233, 64.287627195530014, Math.toRadians(90));
        BlueRight = new Pose2d(-35.04860248750233, 64.287627195530014, Math.toRadians(90));

        BlueWarehouseBarcode = new Pose2d(27.102410493136468, 25.8890852672091, 0);
        BlueWarehouseHub_3 = new Pose2d(-20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        BlueWarehouseHub = new Pose2d(20.37210776664967, 22.922848844623836, 0);
        BlueWarehouseEnd_3 = new Pose2d(-18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        BlueWarehouseEnd = new Pose2d (18.641856780246332, 72.96895702032774, Math.toRadians(90));

        BlueStorageCarousel = new Pose2d ( -0.7903279673816092,  23.597237269651234, Math.toRadians(180));
        BlueStorageBarcode = new Pose2d (4, 24.422398284891067, 0);
        BlueStorageHub_3 = new Pose2d(-25.73856866425003,  -15.932383343462828, Math.toRadians(180));
        BlueStorageHubApproach = new Pose2d ( -15.833455218445653,  -15.833455218445653, 0);
        BlueStorageHub = new Pose2d(-26.733416119168876, -22.11591122428841, 0);
        BlueStorageEnd = new Pose2d ( -21.86613994016752, 31.3381255422029, Math.toRadians(180));



        RedWarehouseBarcode = new Pose2d(-27.102410493136468, 25.8890852672091, 0);
        RedWarehouseHub_3 = new Pose2d(20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        RedWarehouseHub = new Pose2d(-20.37210776664967, 22.922848844623836, 0);
        RedWarehouseEnd_3 = new Pose2d(18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        RedWarehouseEnd = new Pose2d (-18.641856780246332, 72.96895702032774, Math.toRadians(90));

        RedStorageCarousel = new Pose2d (5.1327873025844015, -25.261654154033103, Math.toRadians(180));
        RedStorageBarcode = new Pose2d (-4, 24.422398284891067, 0);
        RedStorageHubApproach = new Pose2d (15.833455218445653,  -15.833455218445653, 0);
        RedStorageHub_3 = new Pose2d(-20.093903824570501, 23.754585428696576, 0);
        RedStorageHub = new Pose2d(20.093903824570501, 23.754585428696576, Math.toRadians(180));
        RedStorageEnd_3 = new Pose2d (26.706247391511965, -27.18861475380927, Math.toRadians(180));
        RedStorageEnd = new Pose2d (-26.706247391511965, -27.18861475380927, 0);
    }


    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        reader = new BarcodeReader();
        webcam.setPipeline(reader);

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

        Positions();


        while (!gamepad1.triangle) {
            telemetry.addData("Left Brightness: ", reader.R1Y);
            telemetry.addData("Middle Brightness: ", reader.R2Y);
            telemetry.addData("Right Brightness: ", reader.R3Y);
            telemetry.addData("Deliver To", reader.getAnalysis());
            telemetry.update();
        }

        config();
        drive.setPoseEstimate(startPose);


        initTrajectories();

        robot.grabberServo.setPosition(1);

        waitForStart();

        if (isStarted()) {

            sleep(waitTime);

            deliverShipment();

            gotoFinish();
        }
    }

    public void gotoFinish()
    {
        switch(alliance) {

            case ("RED"): {


                switch (side) {

                    case ("LEFT"): {

                        drive.followTrajectory(toCarousel);
                        robot.spinnyWheel.setPower(-0.6);
                        spinCarousel();
                        break;
                    }

                    case ("RIGHT"): {

                        drive.followTrajectory(toWarehouseSetup);
                        drive.followTrajectory(toWarehouse);
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
                        .lineToLinearHeading(RedStorageHub)
                        .build();

                switch (side) {

                    case ("LEFT"): {

                        toCarousel = drive.trajectoryBuilder(toShipment.end())
                                .lineToLinearHeading(RedStorageCarousel)
                                .build();
                        break;
                    }

                    case ("RIGHT"): {

                        toWarehouseSetup = drive.trajectoryBuilder(toShipment.end())
                                .lineToLinearHeading(RedStorageHubApproach)
                                .addTemporalMarker(.25, ()-> {
                                    robot.armMotorOne.setPower(-0.2);
                                    robot.armMotorTwo.setPower(-0.2);
                                    robot.armMotorOne.setTargetPosition(-1000);
                                    robot.armMotorTwo.setTargetPosition(-1000);
                                })
                                .build();

                        toWarehouse = drive.trajectoryBuilder(toWarehouseSetup.end())
                                .strafeRight(40)
                                .build();

                        break;
                    }
                }


                break;
            }

            case ("BLUE"): {

                toShipment = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(BlueStorageHub)
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
        int encoderPos = reader.getAnalysis();

        robot.armMotorOne.setPower(-0.2);
        robot.armMotorTwo.setPower(-0.2);
        robot.armMotorOne.setTargetPosition(encoderPos);
        robot.armMotorTwo.setTargetPosition(encoderPos);
        // Drive to position
        drive.followTrajectory(toShipment);

        // Claw actions
        robot.grabberServo.setPosition(-1);
        sleep(50);
        robot.grabberServo.setPosition(1);
        sleep(50);
    }

    private void spinCarousel() {



        switch (alliance) {

            case "BLUE": {
                robot.spinnyWheel.setDirection(DcMotorEx.Direction.REVERSE);
                break;
            }

            case "RED": {

                robot.spinnyWheel.setDirection(DcMotorEx.Direction.FORWARD);
                break;
            }

        }

        // set spinner speed
        robot.spinnyWheel.setVelocity(250);

        // wait 2.5 seconds for the duck to fall
        sleep(1800);
        robot.spinnyWheel.setVelocity(0);

    }

    public Pose2d DetStartPose(int allI, int sideI)
    {
        // Red
        if(allI == 0)
        {
            //Left
            if(sideI == 0)
            {
                return RedLeft;
            }
            return RedRight;
        }
        // Blue
        if(sideI == 0)
        {
            return BlueLeft;
        }
        return BlueRight;
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
            startPose = DetStartPose(allianceIndex, sideIndex);

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

        sleep(500);
        telemetry.clearAll();
        telemetry.addLine("CONFIRMED. READY TO START");
        telemetry.update();
    }
}
