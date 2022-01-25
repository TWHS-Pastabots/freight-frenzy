package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@Autonomous(name = "BlueWarehouse")
public class BlueWarehouse extends LinearOpMode
{
    SampleMecanumDrive drive;

    private Pose2d shippingHub = new Pose2d(19, -18, 0);
    private Pose2d positionOne = new Pose2d(24, 0, Math.toRadians(90));
    private Pose2d positionTwo = new Pose2d(24, 48, Math.toRadians(90));
    private Pose2d positionThree = new Pose2d( 27, -72, Math.toRadians(180));

    private Trajectory toShippingHub;
    private Trajectory returnTrajectory;
    private Trajectory trajectoryOne;
    private Trajectory trajectoryTwo;
    private Trajectory trajectoryThree;

    private final String STORAGE = "Storage";
    private final String NO_STORAGE = "No Storage";
    private String endPoint = STORAGE;

    public void runOpMode()
    {
        RavioliHardware hardware = new RavioliHardware();
        util utilities = new util(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        hardware.init(hardwareMap);

        buildTrajectories();
        utilities.closeClaw();

        configuration();

        waitForStart();
        if(!opModeIsActive()) {return;}

        switch (endPoint)
        {
            case STORAGE:
                utilities.moveArm(-360);
                utilities.wait(2500);
                drive.followTrajectory(toShippingHub);
                utilities.openClaw();
                utilities.wait(500);
                utilities.moveArm(-240);
                drive.followTrajectory(returnTrajectory);
                drive.followTrajectory(trajectoryOne);
                drive.followTrajectory(trajectoryTwo);
                break;
            case NO_STORAGE:
                utilities.moveArm(-360);
                utilities.wait(2500);
                drive.followTrajectory(toShippingHub);
                utilities.wait(500);
                utilities.openClaw();
                utilities.wait(1000);
                drive.followTrajectory(returnTrajectory);
                drive.followTrajectory(trajectoryOne);
        }
    }

    private void buildTrajectories()
    {
        toShippingHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(shippingHub).build();

        returnTrajectory = drive.trajectoryBuilder(toShippingHub.end())
                .lineToSplineHeading(new Pose2d(0, 0, 0)).build();

        trajectoryOne = drive.trajectoryBuilder(returnTrajectory.end())
                .lineToSplineHeading(positionOne).build();

        trajectoryTwo = drive.trajectoryBuilder(trajectoryOne.end())
                .lineToSplineHeading(positionTwo).build();

        trajectoryThree = drive.trajectoryBuilder(trajectoryTwo.end())
                .lineToSplineHeading(positionThree).build();
    }

    private void configuration()
    {
        while (!gamepad1.cross && !isStarted())
        {
            if(gamepad1.right_bumper)
            {
                endPoint = STORAGE;
            }
            else if(gamepad1.left_bumper)
            {
                endPoint = NO_STORAGE;
            }
            telemetry.addData("Endpoint", endPoint);
            telemetry.update();
        }
        telemetry.addData("Status", "Confirmed");
        telemetry.update();
    }
}
