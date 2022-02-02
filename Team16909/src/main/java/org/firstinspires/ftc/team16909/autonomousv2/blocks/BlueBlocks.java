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
    public final Pose2d posBlueStartW = new Pose2d(12, 66.5, Math.toRadians(-90));

    // Blue Pushback
    public final Pose2d posBluePushback = new Pose2d(-36, 0,Math.toRadians(-90));

    //===============================

    // OBJECTIVES

    // Blue Shipping Hub
    public final Pose2d posBlueHubApproach = new Pose2d(-12, 36, Math.toRadians(-90));
    public final Pose2d posBlueHub = new Pose2d(-12, 36, Math.toRadians(-90));

    //Blue Carousel
    public final Pose2d posBlueCarousel = new Pose2d(-60, 48, Math.toRadians(-180));

    //================================

    // PARKING

    // Blue Storage Unit
    public final Pose2d posBlueUnitPark = new Pose2d(-60, 36, Math.toRadians(0));

    // Blue Warehouse
    public final Pose2d posBlueWarehouseApproach = new Pose2d(12, 63, Math.toRadians(-90));
    public final Pose2d posBlueWarehouseEnter = new Pose2d(36, 63, Math.toRadians(-90));
    public final Pose2d posBlueWarehousePark = new Pose2d(36, 51, Math.toRadians(-90));

    //=================================

    // TRAJECTORIES

    TrajectorySequence trajBluePushback = null;
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

    public Pose2d bluePushback(Pose2d start) {
        trajBluePushback = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBluePushback)
                .build();
        drive.followTrajectorySequence(trajBluePushback);
        return posBluePushback;
    }

    public Pose2d blueHub(Pose2d start, int level, int posTrim) {
        trajBlueHubTo = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBlueHubApproach)
                .lineToLinearHeading(posBlueHub)
                .forward(posTrim)
                .build();
        trajBlueHubAway = drive.trajectorySequenceBuilder(trajBlueHubTo.end())
                .lineToLinearHeading(posBlueHubApproach)
                .build();
        robot.grabber.setPosition(0.2);
        action.moveArm(level, 0);
        drive.followTrajectorySequence(trajBlueHubTo);
        robot.grabber.setPosition(0.5);
        action.waitFor(1);
        drive.followTrajectorySequence(trajBlueHubAway);
        action.moveArm(0, 0);
        return posBlueHubApproach;
    }

    public Pose2d blueCarousel(Pose2d start) {
        trajBlueCarousel = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBlueCarousel)
                .build();
        action.moveCarousel(4,-0.5);
        return posBlueCarousel;
    }

    public Pose2d blueUnit(Pose2d start) {
        trajBlueUnit = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBlueUnitPark)
                .build();
        return posBlueUnitPark;
    }

    public Pose2d blueWarehouse(Pose2d start) {
        trajBlueWarehouse = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posBlueWarehouseApproach)
                .lineToLinearHeading(posBlueWarehouseEnter)
                .lineToLinearHeading(posBlueWarehousePark)
                .build();
        return posBlueWarehousePark;
    }
}
