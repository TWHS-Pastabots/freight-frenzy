package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team16910.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16910.hardware.SpaghettiHardware;
import org.firstinspires.ftc.team16910.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.team16910.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(preselectTeleOp = "Auton Test")
public class AutonProblemFixerNOT extends LinearOpMode
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

    public static Pose2d TestPose;

    public static void Positions()
    {
        TestPose = new Pose2d (-40,  0, 0);
    }

    //Trajectories
    private TrajectorySequence test_traj;

    @Override
    public void runOpMode() throws InterruptedException {

        // All Motors
        robot = new SpaghettiHardware();
        robot.init(hardwareMap);

        // Drive Motors
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        robot.leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        robot.rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        robot.rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        robot.armMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Positions();
        test();

        while (!isStarted()) {
            telemetry.addData("Arm Target", target_pos);
            telemetry.update();
        }

        telemetry.addData("Time Delay: ", startWaitTime);
        waitForStart();
        if (!opModeIsActive()) return;
        helpMethods.waitFor(startWaitTime);
        robot.grabberServo.setPosition(1);
        drive.followTrajectorySequence(test_traj);

    }

    private void test()
    {
        test_traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(TestPose)
                .build();
    }


}
