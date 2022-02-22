package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team16910.drive.autonomous.Coordinates;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.security.spec.PSSParameterSpec;

@Autonomous(preselectTeleOp = "Warehouse RED")
public class RedWarehouse extends LinearOpMode
{
    int current_pos = 0;
    //int leftArmOffset = 0;
    boolean justMoved = true;
    ElapsedTime arm_time = null;
    int target_pos = 0;
    int startWaitTime = 0;

    SpaghettiHardware robot = null;
    SampleMecanumDrive drive = null;
    helpMethods helpMethods;
    BarcodeReader reader;
    OpenCvInternalCamera camera;

    public static Pose2d BlueWarehouseBarcode,BlueWarehouseHub_3, BlueWarehouseHub, BlueWarehouseEnd_3, BlueWarehouseEnd;
    public static Pose2d BlueStorageBarcode,BlueStorageCarousel, BlueStorageHub_3, BlueStorageHub, BlueStorageEnd_3, BlueStorageEnd;

    public static Pose2d RedWarehouseBarcode,RedWarehouseHub_3, RedWarehouseHub, RedWarehouseEnd_3, RedWarehouseEnd;
    public static Pose2d RedStorageBarcode, RedStorageCarousel, RedStorageHub_3, RedStorageHub, RedStorageEnd_3, RedStorageEnd;

    public static void Positions()
    {

        BlueWarehouseBarcode = new Pose2d(27.102410493136468, 25.8890852672091, 0);
        BlueWarehouseHub_3 = new Pose2d(-20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        BlueWarehouseHub = new Pose2d(20.37210776664967, 22.922848844623836, 0);
        BlueWarehouseEnd_3 = new Pose2d(-18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        BlueWarehouseEnd = new Pose2d (18.641856780246332, 72.96895702032774, Math.toRadians(90));

        BlueStorageCarousel = new Pose2d (-5.1327873025844015, -25.261654154033103, Math.toRadians(180));
        BlueStorageBarcode = new Pose2d (4, 24.422398284891067, 0);
        BlueStorageHub_3 = new Pose2d(20.093903824570501, 23.754585428696576, 0);
        BlueStorageHub = new Pose2d(-20.093903824570501, 23.754585428696576, Math.toRadians(180));
        BlueStorageEnd_3 = new Pose2d (-26.706247391511965, -27.18861475380927, Math.toRadians(180));
        BlueStorageEnd = new Pose2d (26.706247391511965, -27.18861475380927, 0);



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
    private TrajectorySequence scan_traj;
    private TrajectorySequence hub_traj_3;
    private TrajectorySequence hub_traj;
    private TrajectorySequence warehouse_traj;
    //private TrajectorySequence warehouse_traj_3;

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
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Positions();
        blueWarehouseTraj();

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
        driveToEnd();
    }

    public void deliverFreight()
    {
        int armPose = reader.getAnalysis();

        drive.followTrajectorySequence(scan_traj);

        robot.armMotorOne.setPower(0.2);
        robot.armMotorTwo.setPower(0.2);
        robot.armMotorOne.setTargetPosition(armPose);
        robot.armMotorTwo.setTargetPosition(armPose);

        if(armPose == -4200)
        {
            hub_traj = drive.trajectorySequenceBuilder(scan_traj.end())
                    .lineToLinearHeading(BlueWarehouseHub_3)
                    .build();
            drive.followTrajectorySequence(hub_traj);
        }

        else
        {
            drive.followTrajectorySequence(hub_traj);
        }
    }

    public void driveToEnd()
    {
        drive.followTrajectorySequence(warehouse_traj);
    }


    private void blueWarehouseTraj()
    {
        scan_traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(RedWarehouseBarcode)
                .build();

        hub_traj_3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(RedWarehouseHub_3)
                .build();

        hub_traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(RedWarehouseHub)
                .build();

        /*warehouse_traj_3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(RedWarehouseEnd_3)
                .build();*/

        warehouse_traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(RedWarehouseEnd)
                .build();

    }




}
