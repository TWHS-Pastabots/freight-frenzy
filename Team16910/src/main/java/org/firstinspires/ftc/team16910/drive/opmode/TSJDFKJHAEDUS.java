package org.firstinspires.ftc.team16910.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.drive.autonomous.BarcodeReader;
import org.firstinspires.ftc.team16910.drive.autonomous.helpMethods;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "drive")
public class TSJDFKJHAEDUS extends LinearOpMode {
    public static double DISTANCE = 48; // in

    int current_pos = 0;
    //int leftArmOffset = 0;
    boolean justMoved = true;
    ElapsedTime arm_time = null;
    int target_pos = 0;

    SpaghettiHardware robot = null;
    SampleMecanumDrive drive = null;
    org.firstinspires.ftc.team16910.drive.autonomous.helpMethods helpMethods;
    BarcodeReader reader;
    OpenCvInternalCamera camera;
    int startWaitTime = 0;

    public static Pose2d BlueWarehouseBarcode,BlueWarehouseHub_3, BlueWarehouseHub, BlueWarehouseEnd_3, BlueWarehouseEnd;
    public static Pose2d BlueStorageBarcode, BlueStorageCarouselApproach, BlueStorageCarousel, BlueStorageHub_3, BlueStorageHubApproach, BlueStorageHub, BlueStorageEnd;

    public static Pose2d RedWarehouseBarcode,RedWarehouseHub_3, RedWarehouseHub, RedWarehouseEnd_3, RedWarehouseEnd;
    public static Pose2d RedStorageBarcode, RedStorageCarousel, RedStorageHub_3, RedStorageHub, RedStorageEnd_3, RedStorageEnd;

    public static void Positions()
    {

        BlueWarehouseBarcode = new Pose2d(27.102410493136468, 25.8890852672091, 0);
        BlueWarehouseHub_3 = new Pose2d(-20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        BlueWarehouseHub = new Pose2d(20.37210776664967, 22.922848844623836, 0);
        BlueWarehouseEnd_3 = new Pose2d(-18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        BlueWarehouseEnd = new Pose2d (18.641856780246332, 72.96895702032774, Math.toRadians(90));

        BlueStorageCarouselApproach = new Pose2d(-5.01343, 0, 0);
        BlueStorageCarousel = new Pose2d ( -0.7903279673816092,  23.597237269651234, Math.toRadians(180));
        //BlueStorageBarcode = new Pose2d (4, 24.422398284891067, 0);
        BlueStorageHubApproach = new Pose2d ( -15.833455218445653,  -15.833455218445653, Math.toRadians(180));
        BlueStorageHub_3 = new Pose2d(-25.73856866425003,  -15.932383343462828, Math.toRadians(180));
        BlueStorageHub = new Pose2d(-26.733416119168876, -22.11591122428841, 0);
        //BlueStorageEnd = new Pose2d ( -21.86613994016752, 31.3381255422029, Math.toRadians(180));



        RedWarehouseBarcode = new Pose2d(-27.102410493136468, 25.8890852672091, 0);
        RedWarehouseHub_3 = new Pose2d(20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        RedWarehouseHub = new Pose2d(-20.37210776664967, 22.922848844623836, 0);
        RedWarehouseEnd_3 = new Pose2d(18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        RedWarehouseEnd = new Pose2d (-18.641856780246332, 72.96895702032774, Math.toRadians(90));

        RedStorageCarousel = new Pose2d (5.1327873025844015, -25.261654154033103, Math.toRadians(180));
        RedStorageBarcode = new Pose2d (-4, 24.422398284891067, 0);
        RedStorageHub_3 = new Pose2d(-20.093903824570501, 23.754585428696576, 0);
        RedStorageHub = new Pose2d(20.093903824570501, 23.754585428696576, Math.toRadians(180));
        RedStorageEnd_3 = new Pose2d (26.706247391511965, -27.18861475380927, Math.toRadians(180));
        RedStorageEnd = new Pose2d (-26.706247391511965, -27.18861475380927, 0);
    }

    //Trajectories
    private Trajectory scan_traj;
    private Trajectory hub_approach_traj;
    private Trajectory hub_traj_3;
    private Trajectory hub_traj;
    private Trajectory storage_traj_3;
    private Trajectory storage_traj;
    private Trajectory carousel_approach_traj;
    private Trajectory carousel_traj;
    private Trajectory reverse_traj;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        reader = new BarcodeReader();
        camera.setPipeline(reader);

        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "*Camera could not be opened*");
                telemetry.update();
            }
        });

        // All Motors
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Positions();
        //blueStorageTraj();

        while (!isStarted()) {
            telemetry.addData("Arm Target", target_pos);
            telemetry.update();
        }

        telemetry.addData("Time Delay: ", startWaitTime);
        waitForStart();
        if (!opModeIsActive()) return;

        //this is where the magic starts
        int armPose = reader.getAnalysis();
        helpMethods.waitFor(startWaitTime);
        robot.grabberServo.setPosition(1);

        drive.followTrajectory(carousel_approach_traj);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(carousel_traj);
        helpMethods.waitFor(1);
        robot.spinnyWheel.setPower(-0.6);
        helpMethods.waitFor(1);
        robot.spinnyWheel.setPower(0);

        if(armPose == 4200)
        {
            drive.followTrajectory(hub_approach_traj);
            robot.armMotorOne.setPower(-0.2);
            robot.armMotorTwo.setPower(-0.2);
            robot.armMotorOne.setTargetPosition(armPose);
            robot.armMotorTwo.setTargetPosition(armPose);
            helpMethods.waitFor(2);
            drive.followTrajectory(hub_traj_3);
            helpMethods.waitFor(1);
            robot.grabberServo.setPosition(-1);
            helpMethods.waitFor(1);
            robot.armMotorOne.setPower(0.2);
            robot.armMotorTwo.setPower(0.2);
            robot.grabberServo.setPosition(1);
            robot.armMotorOne.setTargetPosition(0);
            robot.armMotorTwo.setTargetPosition(0);
            drive.followTrajectory(storage_traj_3);
        }

        else
        {
            drive.followTrajectory(hub_approach_traj);
            drive.turn(Math.toRadians(180));
            robot.armMotorOne.setPower(-0.2);
            robot.armMotorTwo.setPower(-0.2);
            robot.armMotorOne.setTargetPosition(armPose);
            robot.armMotorTwo.setTargetPosition(armPose);
            helpMethods.waitFor(2);
            drive.followTrajectory(hub_traj);
            helpMethods.waitFor(1);
            robot.grabberServo.setPosition(-1);
            helpMethods.waitFor(1);
            robot.armMotorOne.setPower(0.2);
            robot.armMotorTwo.setPower(0.2);
            robot.grabberServo.setPosition(1);
            robot.armMotorOne.setTargetPosition(0);
            robot.armMotorTwo.setTargetPosition(0);
            drive.followTrajectory(reverse_traj);
            drive.turn(Math.toRadians(180));
            drive.followTrajectory(storage_traj);
        }
    }

    /*public void spinSpinnyWheel()
    {
        drive.followTrajectorySequence(carousel_traj);

        robot.spinnyWheel.setPower(-0.6);
        helpMethods.waitFor(1);
        drive.followTrajectorySequence(storage_traj);
    }

    public void deliverFreight()
    {
        int armPose = reader.getAnalysis();

        drive.followTrajectorySequence(scan_traj);

        robot.armMotorOne.setPower(-0.2);
        robot.armMotorTwo.setPower(-0.2);
        robot.armMotorOne.setTargetPosition(armPose);
        robot.armMotorTwo.setTargetPosition(armPose);

        if(armPose == 4200)
        {
            hub_traj_3 = drive.trajectorySequenceBuilder(scan_traj.end())
                    .lineToLinearHeading(BlueStorageHub_3)
                    .build();
            drive.followTrajectorySequence(hub_traj_3);
        }

        else
        {
            drive.followTrajectorySequence(hub_traj);
        }
    }

    public void driveToEnd()
    {
        drive.followTrajectorySequence(storage_traj);
    }*/


    private void blueStorageTraj()
    {
        /*scan_traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(BlueStorageBarcode)
                .build();*/
        carousel_approach_traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(BlueStorageCarouselApproach)
                .build();

        carousel_traj = drive.trajectoryBuilder(BlueStorageCarouselApproach)
                .lineToLinearHeading(BlueStorageCarousel)
                .build();

        hub_approach_traj = drive.trajectoryBuilder(BlueStorageCarousel)
                .lineToLinearHeading(BlueStorageHubApproach)
                .build();

        hub_traj_3 = drive.trajectoryBuilder(BlueStorageCarousel)
                .lineToLinearHeading(BlueStorageHubApproach)
                .lineToLinearHeading(BlueStorageHub_3)
                .build();

        hub_traj = drive.trajectoryBuilder(BlueStorageCarousel)
                .lineToLinearHeading(BlueStorageHubApproach)
                .lineToLinearHeading(BlueStorageHub)
                .build();

        storage_traj_3 = drive.trajectoryBuilder(BlueStorageHub_3)
                .lineToLinearHeading(BlueStorageHubApproach)
                .lineToLinearHeading(BlueStorageHub)
                .build();

        reverse_traj = drive.trajectoryBuilder(BlueStorageHub)
                .lineToLinearHeading(BlueStorageHubApproach)
                .build();

        storage_traj = drive.trajectoryBuilder((BlueStorageHubApproach))
                .lineToLinearHeading(BlueStorageHub)
                .build();

    }




    /*public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d();

        drive.setPoseEstimate(startPose);

        Trajectory skjfkjds = drive.trajectoryBuilder(new Pose2d(), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d( 30, 0, 0)).build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(skjfkjds);
        }*/
    }

