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

import java.security.spec.PSSParameterSpec;

@Autonomous(preselectTeleOp = "Warehouse BLUE")
public class RedWarehouse extends LinearOpMode
{
    int current_pos = 0;
    //int leftArmOffset = 0;
    boolean justMoved = true;
    ElapsedTime arm_time = null;
    int target_pos = 0;

    SpaghettiHardware robot = null;
    SampleMecanumDrive drive = null;
    helpMethods helpMethods;

    //Positions
    private final Pose2d move_Forward = new Pose2d(-9, 0, 0);
    private final Pose2d scan_pos = new Pose2d(15.792390179769374, 0, Math.toRadians(180));
    private final Pose2d hub_pos = new Pose2d(15.792390179769374, -23.07919092716424, Math.toRadians(180));
    private final Pose2d carousel_approach = new Pose2d(13.579214625984648, -66.8355575585923, Math.toRadians(180));
    private final Pose2d carousel_pos = new Pose2d(0.256660589321775, -66.78512696747305, Math.toRadians(351.6687646887457-180));
    private final Pose2d warehouse_pos = new Pose2d(32.74436593969421, 60.00664056942736, Math.toRadians(270));

    //Trajectories
    private TrajectorySequence scan_traj;
    private TrajectorySequence hub_traj;
    private TrajectorySequence carousel_traj;
    private TrajectorySequence warehouse_traj;

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

        //Danny RUN_TO_POSITION Code
       /* while (robot.armEncoder.getCurrentPosition() > encoderPos) {
            for (DcMotorEx motor : robot.motorArms) motor.setPower(-.5);
            telemetry.addData("Arm Position: ", robot.armEncoder.getCurrentPosition());
            telemetry.update(); */

        //Rajiv RUN_TO_POSITION Code
        /*if (armUp == 1 && armTime.time() >= 40)
        {
            robot.rightArm.setTargetPosition(170);
            robot.leftArm.setTargetPosition(170);
//            robot.rightArm.setTargetPosition(robot.rightArm.getCurrentPosition() + 10);
//            robot.leftArm.setTargetPosition(robot.leftArm.getCurrentPosition() + 10);

            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.leftArm.setPower(0.6);
            robot.rightArm.setPower(0.6);

            armTime.reset();*/

        // Action Method Init
        //action = new helpMethods(robot);

        redWarehouseTraj();

        while (!isStarted())
        {
            telemetry.addData("Arm Target", target_pos);
            telemetry.update();
        }

        waitForStart();
        if (!opModeIsActive()) return;
        robot.grabberServo.setPosition(1);
        robot.leftFront.setPower(-.3);
        robot.leftRear.setPower(-.3);
        robot.rightFront.setPower(-.3);
        robot.rightRear.setPower(-.3);
        helpMethods.waitFor(.3);
        drive.followTrajectorySequence(scan_traj);
        helpMethods.waitFor(1);
        drive.followTrajectorySequence(hub_traj);

        robot.armMotorOne.setPower(-0.6);
        robot.armMotorTwo.setPower(-0.6);

        robot.armMotorOne.setTargetPosition(-150);
        robot.armMotorTwo.setTargetPosition(-150);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*robot.armMotorOne.setPower(-0.5);
        robot.armMotorTwo.setPower(-0.5);
        helpMethods.waitFor(1.5);
        robot.armMotorOne.setPower(0);
        robot.armMotorTwo.setPower(0);*/

        /*
        robot.armMotorOne.setTargetPosition(60);
        robot.armMotorTwo.setTargetPosition(60);
        robot.armMotorOne.setPower(0.5);
        robot.armMotorTwo.setPower(-0.5);
        */

        helpMethods.waitFor(2);
        robot.grabberServo.setPosition(-1);
        helpMethods.waitFor(1);
        robot.grabberServo.setPosition(1);
        /*
        robot.armMotorOne.setTargetPosition(10);
        robot.armMotorTwo.setTargetPosition(10);
        robot.armMotorOne.setPower(-0.6);
        robot.armMotorOne.setPower(0.6);
*/
        //helpMethods.waitFor(1);
        drive.followTrajectorySequence(carousel_traj);
        robot.leftSpinnyWheel.setPower(0.6);
        robot.rightSpinnyWheel.setPower(0.6);
        helpMethods.waitFor(3);
        robot.leftSpinnyWheel.setPower(0);
        robot.rightSpinnyWheel.setPower(0);

        robot.armMotorOne.setPower(-0.6);
        robot.armMotorTwo.setPower(-0.6);

        robot.armMotorOne.setTargetPosition(10);
        robot.armMotorTwo.setTargetPosition(10);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.followTrajectorySequence(warehouse_traj);

    }
    /*
        private int armPosition(int level)
        {
            switch (level)
            case 1:
            {

            }

        }

     */
    private void redWarehouseTraj()
    {
        scan_traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                //.lineToLinearHeading(move_Forward)
                .turn(Math.toRadians(180))
                .lineToLinearHeading(scan_pos)
                .build();
        hub_traj = drive.trajectorySequenceBuilder(scan_traj.end())
                .lineToLinearHeading(hub_pos)
                .build();
        /*carousel_traj = drive.trajectorySequenceBuilder(hub_traj.end())
                .lineToLinearHeading(carousel_approach)
                .lineToLinearHeading(carousel_pos)
                .build();*/
        warehouse_traj = drive.trajectorySequenceBuilder(carousel_traj.end())
                //.lineToLinearHeading(carousel_approach)
                .turn(Math.toRadians(90))
                .lineToLinearHeading(warehouse_pos)
                .build();
    }




}
