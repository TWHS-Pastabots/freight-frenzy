package org.firstinspires.ftc.team15021.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team15021.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team15021.hardware.RavioliHardware;

@Autonomous(name = "RedCarousel")
public class RedCarousel extends LinearOpMode
{
    SampleMecanumDrive drive;

    private Pose2d shippingHub = new Pose2d(17.5, -29, 0);
    private Pose2d positionOne = new Pose2d(4, 20, Math.toRadians(0));
    private Pose2d positionTwo = new Pose2d(29, 21, Math.toRadians(0));

    private Trajectory toShippingHub;
    private Trajectory trajectoryOne;
    private Trajectory trajectoryTwo;

    private final String STORAGE = "Storage";
    private final String NO_STORAGE = "No Storage";
    private String endPoint = STORAGE;

    public void runOpMode()
    {
        RavioliHardware hardware = new RavioliHardware();
        util utilities = new util(hardware);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(-90)));
        hardware.init(hardwareMap);

        buildTrajectories();
        utilities.closeClaw();

        configuration();

        waitForStart();
        if(!opModeIsActive()) {return;}

        switch (endPoint)
        {
            case STORAGE:
                utilities.wait(8000);
                utilities.moveArm(-360);
                utilities.wait(3000);
                drive.followTrajectory(toShippingHub);
                utilities.wait(500);
                utilities.openClaw();
                utilities.wait(1000);
                drive.followTrajectory(trajectoryOne);
                utilities.spinRedCarousel();
                utilities.moveArm(-240);
                drive.followTrajectory(trajectoryTwo);
                break;
            case NO_STORAGE:
                utilities.moveArm(-100);
                drive.followTrajectory(toShippingHub);
                utilities.wait(500);
                utilities.openClaw();
                utilities.wait(1000);
                drive.followTrajectory(trajectoryOne);
                utilities.spinCarousel();
        }
    }

    private void buildTrajectories()
    {
        toShippingHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(shippingHub).build();

        trajectoryOne = drive.trajectoryBuilder(toShippingHub.end())
                .lineToLinearHeading(positionOne).build();

        trajectoryTwo = drive.trajectoryBuilder(trajectoryOne.end())
                .lineToLinearHeading(positionTwo).build();
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
