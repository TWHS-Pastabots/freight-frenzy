package org.firstinspires.ftc.team16909.autonomousv2.blocks;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.team16909.autonomousv2.Actions;
import org.firstinspires.ftc.team16909.drive.SampleMecanumDrive;
import org.firstinspires.ftc.team16909.hardware.FettuccineHardware;
import org.firstinspires.ftc.team16909.trajectorysequence.TrajectorySequence;

public class RedBlocks {

    private SampleMecanumDrive drive = null;
    private FettuccineHardware robot = null;
    private Actions action = null;

    // START

    // Blue Starting Positions
    public final Pose2d posRedStartC = new Pose2d(-36, -66.5, Math.toRadians(90));
    public final Pose2d posRedStartW = new Pose2d(12, -66.5, Math.toRadians(90));

    // Blue Pushback
    public final Pose2d posRedPushback = new Pose2d(-36, 0,Math.toRadians(90));

    //===============================

    // OBJECTIVES

    // Blue Shipping Hub
    public final Pose2d posRedHubApproach = new Pose2d(-12, -36, Math.toRadians(90));
    public final Pose2d posRedHub = new Pose2d(-12, -36, Math.toRadians(90));

    //Blue Carousel
    public final Pose2d posRedCarousel = new Pose2d(-60, -48, Math.toRadians(180));

    //================================

    // PARKING

    // Blue Storage Unit
    public final Pose2d posRedUnitPark = new Pose2d(-60, -36, Math.toRadians(0));

    // Blue Warehouse
    public final Pose2d posRedWarehouseApproach = new Pose2d(12, -63, Math.toRadians(90));
    public final Pose2d posRedWarehouseEnter = new Pose2d(36, -63, Math.toRadians(90));
    public final Pose2d posRedWarehousePark = new Pose2d(36, -51, Math.toRadians(90));

    //=================================

    // TRAJECTORIES

    TrajectorySequence trajRedPushback = null;
    TrajectorySequence trajRedHubTo = null;
    TrajectorySequence trajRedHubAway = null;
    TrajectorySequence trajRedCarousel = null;
    TrajectorySequence trajRedUnit = null;
    TrajectorySequence trajRedWarehouse = null;

    public RedBlocks(SampleMecanumDrive drive, FettuccineHardware robot, Actions action) {
        this.drive = drive;
        this.robot = robot;
        this.action = action;
    }

    public Pose2d getStart(@NonNull String start) {
        if (start.equals("RedC")) return posRedStartC;
        return posRedStartW;
    }

    public Pose2d RedPushback(Pose2d start) {
        trajRedPushback = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posRedPushback)
                .build();
        drive.followTrajectorySequence(trajRedPushback);
        return posRedPushback;
    }

    public Pose2d RedHub(Pose2d start, int level, int posTrim) {
        trajRedHubTo = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posRedHubApproach)
                .lineToLinearHeading(posRedHub)
                .forward(posTrim)
                .build();
        trajRedHubAway = drive.trajectorySequenceBuilder(trajRedHubTo.end())
                .lineToLinearHeading(posRedHubApproach)
                .build();
        robot.grabber.setPosition(0.2);
        action.moveArm(level, 0);
        drive.followTrajectorySequence(trajRedHubTo);
        robot.grabber.setPosition(0.5);
        action.waitFor(1);
        drive.followTrajectorySequence(trajRedHubAway);
        action.moveArm(0, 0);
        return posRedHubApproach;
    }

    public Pose2d RedCarousel(Pose2d start) {
        trajRedCarousel = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posRedCarousel)
                .build();
        action.moveCarousel(4, 0.5);
        return posRedCarousel;
    }

    public Pose2d RedUnit(Pose2d start) {
        trajRedUnit = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posRedUnitPark)
                .build();
        return posRedUnitPark;
    }

    public Pose2d RedWarehouse(Pose2d start) {
        trajRedWarehouse = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(posRedWarehouseApproach)
                .lineToLinearHeading(posRedWarehouseEnter)
                .lineToLinearHeading(posRedWarehousePark)
                .build();
        return posRedWarehousePark;
    }

}