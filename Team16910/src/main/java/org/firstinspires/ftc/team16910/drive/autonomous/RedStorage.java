package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(preselectTeleOp = "Storage BLUE")
public class RedStorage extends LinearOpMode
{
    int current_pos = 0;
    //int leftArmOffset = 0;
    boolean justMoved = true;
    ElapsedTime arm_time = null;
    int target_pos = 0;

    SpaghettiHardware robot = null;
    SampleMecanumDrive drive = null;
    helpMethods helpMethods;
    BarcodeReader reader;
    OpenCvInternalCamera camera;
    int startWaitTime = 0;

    public static Pose2d BlueWarehouseBarcode,BlueWarehouseHub_3, BlueWarehouseHub, BlueWarehouseEnd_3, BlueWarehouseEnd;
    public static Pose2d BlueStorageBarcode,BlueStorageCarousel, BlueStorageCarouselApproach, BlueStorageHub_3, BlueStorageHubApproach, BlueStorageHub, BlueStorageEnd, BlueStorageEndApproach;

    public static Pose2d RedWarehouseBarcode,RedWarehouseHub_3, RedWarehouseHub, RedWarehouseEnd_3, RedWarehouseEnd;
    public static Pose2d RedStorageBarcode, RedStorageCarousel, RedStorageHub_3, RedStorageHub, RedStorageEndApproach, RedStorageCarouselApproach, RedStorageHubApproach, RedStorageEnd_3, RedStorageEnd;

    public static void Positions()
    {

        BlueWarehouseBarcode = new Pose2d(27.102410493136468, 25.8890852672091, 0);
        BlueWarehouseHub_3 = new Pose2d(-20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        BlueWarehouseHub = new Pose2d(20.37210776664967, 22.922848844623836, 0);
        BlueWarehouseEnd_3 = new Pose2d(-18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        BlueWarehouseEnd = new Pose2d (18.641856780246332, 72.96895702032774, Math.toRadians(90));

        BlueStorageCarousel = new Pose2d ( -7.094957356362559,   25.5, Math.toRadians(180));
        BlueStorageCarouselApproach = new Pose2d ( -12.25018219910173,  0, 0);
        //BlueStorageBarcode = new Pose2d (3, 0 ,0);
        //BlueStorageBarcode = new Pose2d (4, 24.422398284891067, 0);
        BlueStorageHub_3 = new Pose2d( -19.98260633911189,   -20.71296377798099, Math.toRadians(180));
        BlueStorageHubApproach = new Pose2d ( -18.581288788795764,  -19.55038475690724, Math.toRadians(180));
        BlueStorageHub = new Pose2d( -25.424164654447086,  -19.167914849926184, 0);
        BlueStorageEnd = new Pose2d (  -26.644110760384077, 33.05485678336123, 0);
        BlueStorageEndApproach = new Pose2d ( -5, 0, 0);



        RedWarehouseBarcode = new Pose2d(-27.102410493136468, 25.8890852672091, 0);
        RedWarehouseHub_3 = new Pose2d(20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        RedWarehouseHub = new Pose2d(-20.37210776664967, 22.922848844623836, 0);
        RedWarehouseEnd_3 = new Pose2d(18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        RedWarehouseEnd = new Pose2d (-18.641856780246332, 72.96895702032774, Math.toRadians(90));

        RedStorageCarousel = new Pose2d ( -2.094957356362559,   -17.5,Math.toRadians(90) );
        RedStorageCarouselApproach = new Pose2d ( -12.25018219910173,  0, 0);
        //BlueStorageBarcode = new Pose2d (3, 0 ,0);
        //BlueStorageBarcode = new Pose2d (4, 24.422398284891067, 0);
        RedStorageHub_3 = new Pose2d( -19.98260633911189,   20.71296377798099, Math.toRadians(180));
        RedStorageHubApproach = new Pose2d ( -15.581288788795764,  27.55038475690724, Math.toRadians(180));
        RedStorageHub = new Pose2d( -16.424164654447086,  25.167914849926184, 0);
        RedStorageEnd = new Pose2d (  -29.644110760384077, -29.05485678336123, 0);
        RedStorageEndApproach = new Pose2d ( -5, 0, 0);
    }

    //Trajectories
    private Trajectory scan_traj;
    private Trajectory hub_traj_3;
    private Trajectory hub_traj;
    private Trajectory hub_approach_traj;
    private Trajectory storage_approach_traj;
    private Trajectory storage_traj;
    //private Trajectory warehouse_traj_3;
    private Trajectory carousel_traj;
    private Trajectory carousel_approach_traj;

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

        Positions();
        redStorageTraj();

        robot.grabberServo.setPosition(1);

        while (!isStarted()) {
            telemetry.addData("Arm Target", target_pos);
            telemetry.update();
        }

        telemetry.addData("Time Delay: ", startWaitTime);
        waitForStart();
        if (!opModeIsActive()) return;
        helpMethods.waitFor(startWaitTime);
        int armPose = reader.getAnalysis();
        drive.followTrajectory(carousel_approach_traj);
        drive.followTrajectory(carousel_traj);
        robot.spinnyWheel.setPower(-0.6);
        helpMethods.waitFor(3);
        robot.spinnyWheel.setPower(0);

        //drive.followTrajectory(scan_traj);
        drive.followTrajectory(hub_approach_traj);

        robot.armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("armPose", armPose);
        telemetry.update();

        robot.armMotorOne.setTargetPosition(armPose);
        robot.armMotorTwo.setTargetPosition(armPose);
        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorOne.setPower(0.8);
        robot.armMotorTwo.setPower(0.8);
        robot.armMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(armPose == -210)
        {
            hub_traj_3 = drive.trajectoryBuilder(hub_approach_traj.end())
                    .lineToLinearHeading(RedStorageHub_3)
                    .build();
            drive.followTrajectory(hub_traj_3);
            helpMethods.waitFor(2);
            robot.grabberServo.setPosition(0);
            helpMethods.waitFor(1);
            robot.grabberServo.setPosition(1);
            helpMethods.waitFor(1);

            telemetry.addData("armPose", target_pos);
            telemetry.update();

            robot.armMotorOne.setTargetPosition(50);
            robot.armMotorTwo.setTargetPosition(50);
            robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotorOne.setPower(0.3);
            robot.armMotorTwo.setPower(0.3);

            storage_approach_traj = drive.trajectoryBuilder(hub_traj_3.end())
                    .lineToLinearHeading(RedStorageEndApproach)
                    .build();

            helpMethods.waitFor(1);

            drive.followTrajectory(storage_approach_traj);

            storage_traj = drive.trajectoryBuilder(storage_approach_traj.end())
                    .lineToLinearHeading(RedStorageEnd)
                    .build();

            helpMethods.waitFor(1);
            drive.followTrajectory(storage_traj);
        }
        else
        {
            drive.followTrajectory(hub_traj);

            helpMethods.waitFor(3);
            robot.grabberServo.setPosition(0);

            robot.armMotorOne.setTargetPosition(50);
            robot.armMotorTwo.setTargetPosition(50);
            robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotorOne.setPower(.3);
            robot.armMotorTwo.setPower(.3);

            storage_approach_traj = drive.trajectoryBuilder(hub_traj.end())
                    .lineToLinearHeading(RedStorageEndApproach)
                    .build();
            drive.followTrajectory(storage_approach_traj);


            storage_traj = drive.trajectoryBuilder(storage_approach_traj.end())
                    .lineToLinearHeading(RedStorageEnd)
                    .build();

            helpMethods.waitFor(1);
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


    private void redStorageTraj()
    {
        carousel_approach_traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(RedStorageCarouselApproach)
                .build();

        carousel_traj = drive.trajectoryBuilder(carousel_approach_traj.end())
                .lineToLinearHeading(RedStorageCarousel)
                .build();

        /*scan_traj = drive.trajectoryBuilder(carousel_traj.end())
                .lineToLinearHeading(BlueStorageBarcode)
                .build();*/

        hub_approach_traj = drive.trajectoryBuilder(carousel_traj.end())
                .lineToLinearHeading(RedStorageHubApproach)
                .build();

        hub_traj_3 = drive.trajectoryBuilder(hub_approach_traj.end())
                .lineToLinearHeading(RedStorageHub_3)
                .build();

        hub_traj = drive.trajectoryBuilder(hub_approach_traj.end())
                .lineToLinearHeading(RedStorageHub)
                .build();


        /*warehouse_traj_3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(BlueWarehouseEnd_3)
                .build();*/





    }




}
