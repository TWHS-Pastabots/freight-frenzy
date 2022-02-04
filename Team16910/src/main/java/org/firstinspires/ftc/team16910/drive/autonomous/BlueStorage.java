package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.trajectorysequence.TrajectorySequence;

@Autonomous(preselectTeleOp = "Storage BLUE")
public class BlueStorage extends LinearOpMode
{
    int current_pos = 0;
    //int leftArmOffset = 0;
    boolean justMoved = true;
    ElapsedTime arm_time = null;
    int target_pos = 0;

    SpaghettiHardware robot = null;
    SampleMecanumDrive drive = null;
    helpMethods helpMethods;

    /*
    path:
    close grabber
    move forward just a bit
    move right
    spin carousel
    move to carousel approach
    move to hub_pos
    move arm forward
    release grabber
    move arm backward
    rotate left 90
    move to warehouse pos
     */


    private final Pose2d move_Forward = new Pose2d(-8.013438741993097, -0.028127556611040036, 0);
    //private final Pose2d carousel_approach = new Pose2d(-8.013438741993097, -10.324558386544119, Math.toRadians(180));
    private final Pose2d carousel_pos = new Pose2d(-5.06136975539866, 20.991465591563607, Math.toRadians(180));
    private final Pose2d hub_approach = new Pose2d(-21.102410493136468, 25.8890852672091, Math.toRadians(180));
    private final Pose2d hub_pos = new Pose2d(-19.37210776664967, -20.922848844623836, Math.toRadians(180));
    private final Pose2d warehouse_approach = new Pose2d(-18.107768895372946, -18.16996569873317, Math.toRadians(270));
    private final Pose2d storage_pos = new Pose2d(-8.013438741993097, 30.991465591563607, Math.toRadians(270));

    //Positions

    //Trajectories
    private TrajectorySequence carousel_traj;
    private TrajectorySequence hub_traj;
    private TrajectorySequence storage_traj;

    @Override
    public void runOpMode() throws InterruptedException
    {

        // All Motors
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        blueStorageTraj();

        while (!isStarted())
        {
            telemetry.addData("Arm Target", target_pos);
            telemetry.update();
        }

        waitForStart();
        if (!opModeIsActive()) return;
        robot.grabberServo.setPosition(1);
        helpMethods.waitFor(1);
        //drive.followTrajectorySequence(carousel_approach);
        drive.followTrajectorySequence(carousel_traj);
        //spin the thing
        helpMethods.waitFor(1);
        robot.leftSpinnyWheel.setPower(0.6);
        robot.rightSpinnyWheel.setPower(0.6);
        helpMethods.waitFor(3);
        robot.leftSpinnyWheel.setPower(0);
        robot.rightSpinnyWheel.setPower(0);
        //move to driving the stuff
        drive.followTrajectorySequence(hub_traj);
        //bringing the arm up
        /*
        robot.armMotorOne.setPower(-0.5);
        robot.armMotorTwo.setPower(-0.5);
        helpMethods.waitFor(1.4);
        robot.armMotorOne.setPower(0);
        robot.armMotorTwo.setPower(0);
        helpMethods.waitFor(1.7);
        robot.grabberServo.setPosition(-1);
        //waiting
        helpMethods.waitFor(2);
        //bringing the arm down
        robot.armMotorOne.setPower(0.5);
        robot.armMotorOne.setPower(0.5);
        helpMethods.waitFor(0.5);
        robot.armMotorOne.setPower(0);
        robot.armMotorTwo.setPower(0);
        */
        robot.armMotorOne.setPower(-0.6);
        robot.armMotorTwo.setPower(-0.6);

        robot.armMotorOne.setTargetPosition(-140);
        robot.armMotorTwo.setTargetPosition(-140);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        helpMethods.waitFor(2);
        robot.grabberServo.setPosition(-1);
        helpMethods.waitFor(1);
        robot.grabberServo.setPosition(1);

        robot.armMotorOne.setPower(0.6);
        robot.armMotorTwo.setPower(0.6);

        robot.armMotorOne.setTargetPosition(10);
        robot.armMotorTwo.setTargetPosition(10);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectorySequence(storage_traj);

    }
    private void blueStorageTraj()
    {
        carousel_traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(move_Forward)
                .turn(Math.toRadians(180))
                //.lineToLinearHeading(carousel_approach)
                .lineToLinearHeading(carousel_pos)
                .build();
        hub_traj = drive.trajectorySequenceBuilder(carousel_traj.end())
                .lineToLinearHeading(hub_approach)
                .lineToLinearHeading(hub_pos)
                .build();
        storage_traj = drive.trajectorySequenceBuilder(hub_traj.end())
                .turn(Math.toRadians(90))
                .lineToLinearHeading(storage_pos)
                .build();
    }




}
