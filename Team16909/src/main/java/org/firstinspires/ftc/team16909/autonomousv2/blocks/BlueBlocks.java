package org.firstinspires.ftc.team16909.autonomousv2.blocks;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.team16909.autonomousv2.Actions;
import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;
import org.firstinspires.ftc.team16909.trajectorysequence.TrajectorySequence;

public class BlueBlocks {

    private SampleMecanumDrive drive = null;
    private FettuccineHardware robot = null;
    private Actions action = null;

    // START

    // Blue Starting Positions
    public final Pose2d posBlueStartC = new Pose2d(-36, 66.5, Math.toRadians(-90));
    public final Pose2d posBlueStartW = new Pose2d(11, 66.5, Math.toRadians(-90));

    // Blue Pushback
    public final Pose2d posBluePushbackC = new Pose2d(-36, 60, Math.toRadians(-90));
    public final Pose2d posBluePushbackW = new Pose2d(11, 60, Math.toRadians(-90));

    //===============================

    // OBJECTIVES

    // Blue Shipping Hub
    public final Pose2d posBlueHubApproach = new Pose2d(-15, 60, Math.toRadians(-90));
    public final Pose2d posBlueHub = new Pose2d(-15, 42, Math.toRadians(0));
    public final Pose2d posBlueHubLow = new Pose2d(-15, 38, Math.toRadians(-180));

    //Blue Carousel
    public final Pose2d posBlueCarousel = new Pose2d(-60, 48, Math.toRadians(0));

    //================================

    // PARKING

    // Blue Storage Unit
    public final Pose2d posBlueUnitPark = new Pose2d(-63, 36, Math.toRadians(0));

    // Blue Warehouse
    public final Pose2d posBlueWarehouseApproach = new Pose2d(11, 68.5, Math.toRadians(-90));
    public final Pose2d posBlueWarehouseEnter = new Pose2d(52, 68.5, Math.toRadians(-90));
    public final Pose2d posBlueWarehousePark = new Pose2d(52, 51, Math.toRadians(-90));

    //=================================

    // TRAJECTORIES

    TrajectorySequence trajBluePushbackC = null;
    TrajectorySequence trajBluePushbackW = null;
    TrajectorySequence trajBlueHubTo = null;
    TrajectorySequence trajBlueHubAway = null;
    TrajectorySequence trajBlueCarousel = null;
    TrajectorySequence trajBlueUnit = null;
    TrajectorySequence trajBlueWarehouse = null;

    public BlueBlocks(SampleMecanumDrive drive, FettuccineHardware robot, Actions action) {
        this.drive = drive;
        this.robot = robot;
        this.action = action;
    }

    public Pose2d getStart(@NonNull String start) {
        if (start.equals("BlueC")) return posBlueStartC;
        return posBlueStartW;
    }

    public Pose2d BluePushbackC(Pose2d start) {
        trajBluePushbackC = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBluePushbackC)
                .build();
        drive.followTrajectorySequence(trajBluePushbackC);
        return posBluePushbackC;
    }

    public Pose2d BluePushbackW(Pose2d start) {
        trajBluePushbackW = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBluePushbackW)
                .build();
        drive.followTrajectorySequence(trajBluePushbackW);
        return posBluePushbackW;
    }

    public Pose2d BlueHub(Pose2d start, int level, int posTrim) {
        if (level > 1) {
            trajBlueHubTo = drive.trajectorySequenceBuilder(start)
                    .lineToLinearHeading(posBlueHubApproach)
                    .lineToLinearHeading(posBlueHub)
                    .forward(posTrim)
                    .build();
        } else {
            trajBlueHubTo = drive.trajectorySequenceBuilder(start)
                    .lineToLinearHeading(posBlueHubApproach)
                    .lineToLinearHeading(posBlueHubLow)
                    .forward(posTrim)
                    .build();
        }
        trajBlueHubAway = drive.trajectorySequenceBuilder(trajBlueHubTo.end())
                .lineToLinearHeading(posBlueHubApproach)
                .build();
        robot.grabber.setPosition(0.5);
        action.waitFor(0.5);
        action.moveArm(level, 0);
        action.waitFor(2);
        drive.followTrajectorySequence(trajBlueHubTo);
        robot.grabber.setPosition(0);
        action.waitFor(1);
        drive.followTrajectorySequence(trajBlueHubAway);
        action.moveArm(0, 0);
        action.waitFor(1);
        return posBlueHubApproach;
    }

    public Pose2d BlueCarousel(Pose2d start) {
        trajBlueCarousel = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBlueCarousel)
                .build();
        drive.followTrajectorySequence(trajBlueCarousel);
        action.moveCarousel(4,-0.4);
        return posBlueCarousel;
    }

    public Pose2d BlueUnit(Pose2d start) {
        trajBlueUnit = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBlueUnitPark)
                .build();
        drive.followTrajectorySequence(trajBlueUnit);
        return posBlueUnitPark;
    }

    public Pose2d BlueWarehouse(Pose2d start) {
        trajBlueWarehouse = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBlueWarehouseApproach)
                .lineToLinearHeading(posBlueWarehouseEnter)
                .lineToLinearHeading(posBlueWarehousePark)
                .build();
        drive.followTrajectorySequence(trajBlueWarehouse);
        return posBlueWarehousePark;
    }
}
