package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(preselectTeleOp = "Storage RED")
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
    public static Pose2d BlueStorageBarcode,BlueStorageCarousel, BlueStorageHub_3, BlueStorageHubApproach, BlueStorageHub, BlueStorageEnd_3, BlueStorageEnd;

    public static Pose2d RedWarehouseBarcode,RedWarehouseHub_3, RedWarehouseHub, RedWarehouseEnd_3, RedWarehouseEnd;
    public static Pose2d RedStorageBarcode, RedStorageCarousel, RedStorageHub_3, RedStorageHubApproach, RedStorageHub, RedStorageEnd;

    public static void Positions()
    {

        BlueWarehouseBarcode = new Pose2d(27.102410493136468, 25.8890852672091, 0);
        BlueWarehouseHub_3 = new Pose2d(-20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        BlueWarehouseHub = new Pose2d(20.37210776664967, 22.922848844623836, 0);
        BlueWarehouseEnd_3 = new Pose2d(-18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        BlueWarehouseEnd = new Pose2d (18.641856780246332, 72.96895702032774, Math.toRadians(90));

        BlueStorageCarousel = new Pose2d ( -0.7903279673816092,  23.597237269651234, Math.toRadians(180));
        BlueStorageBarcode = new Pose2d (4, 24.422398284891067, 0);
        BlueStorageHub_3 = new Pose2d(-15.932383343462828,  -25.73856866425003, Math.toRadians(180));
        BlueStorageHubApproach = new Pose2d ( -15.833455218445653,  -25.501051561159723, 0);
        BlueStorageHub = new Pose2d(-26.733416119168876, -22.11591122428841, 0);
        BlueStorageEnd_3 = new Pose2d (-26.706247391511965, -27.18861475380927, Math.toRadians(180));
        BlueStorageEnd = new Pose2d ( -21.86613994016752, 31.3381255422029, Math.toRadians(180));



        RedWarehouseBarcode = new Pose2d(-27.102410493136468, 25.8890852672091, 0);
        RedWarehouseHub_3 = new Pose2d(20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        RedWarehouseHub = new Pose2d(-20.37210776664967, 22.922848844623836, 0);
        RedWarehouseEnd_3 = new Pose2d(18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        RedWarehouseEnd = new Pose2d (-18.641856780246332, 72.96895702032774, Math.toRadians(90));

        RedStorageBarcode = new Pose2d (0, 0, 0);
        RedStorageHub_3 = new Pose2d(-14.192274927952655,  26.11443752408844, Math.toRadians(180));
        RedStorageHubApproach = new Pose2d ( -9.536640483060621,  27.101316135577758, 0);
        RedStorageHub = new Pose2d(-23.422276536545855, 25.96772301835214, 0);
        RedStorageCarousel = new Pose2d ( -4.925757405507023,   -16.651212706329115, Math.toRadians(90));
        RedStorageEnd = new Pose2d ( -25.86454073661928, -18.712883596256887, Math.toRadians(90));
    }

    //Trajectories
    private TrajectorySequence scan_traj;
    private TrajectorySequence hub_traj_3;
    private TrajectorySequence hub_traj;
    private TrajectorySequence storage_traj;
    //private TrajectorySequence warehouse_traj_3;
    private TrajectorySequence carousel_traj;

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
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Positions();
        redWarehouseTraj();

        while (!isStarted()) {
            telemetry.addData("Arm Target", target_pos);
            telemetry.update();
        }

        telemetry.addData("Time Delay: ", startWaitTime);
        waitForStart();
        if (!opModeIsActive()) return;
        helpMethods.waitFor(startWaitTime);
        robot.grabberServo.setPosition(1);
        deliverFreight();
        helpMethods.waitFor(1);
        spinSpinnyWheel();
        helpMethods.waitFor(1);
        driveToEnd();
    }

    public void spinSpinnyWheel()
    {
        drive.followTrajectorySequence(carousel_traj);

        robot.spinnyWheel.setPower(-0.6);
        helpMethods.waitFor(1);
    }

    public void deliverFreight()
    {
        int armPose = reader.getAnalysis();

        drive.followTrajectorySequence(scan_traj);

        robot.armMotorOne.setPower(-0.2);
        robot.armMotorTwo.setPower(-0.2);
        robot.armMotorOne.setTargetPosition(armPose);
        robot.armMotorTwo.setTargetPosition(armPose);

        if(armPose == 3900)
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
    }


    private void redWarehouseTraj()
    {
        scan_traj = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(RedStorageBarcode)
                .build();

        hub_traj_3 = drive.trajectorySequenceBuilder(scan_traj.end())
                .lineToLinearHeading(RedStorageHub_3)
                .build();

        hub_traj = drive.trajectorySequenceBuilder(scan_traj.end())
                .lineToLinearHeading(RedStorageHub)
                .build();

        carousel_traj = drive.trajectorySequenceBuilder(hub_traj.end())
                .lineToLinearHeading(RedStorageCarousel)
                .build();


        /*warehouse_traj_3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(BlueWarehouseEnd_3)
                .build();*/

        storage_traj = drive.trajectorySequenceBuilder(carousel_traj.end())
                .lineToLinearHeading(RedStorageEnd)
                .build();

    }




}
