package org.firstinspires.ftc.team16909.autonomousv2.waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class RedWaypoints {

    // Red Hub
    private final Pose2d posHub = new Pose2d(17.977, 30.689, Math.toRadians(0));

    // Red Carousel
    private final Pose2d posRedPushback = new Pose2d(4.7063, 0,Math.toRadians(0));
    private final Pose2d posRedCarousel = new Pose2d(6.9063, 17.9322, Math.toRadians(90));
    private final Pose2d posRedUnitPark = new Pose2d(32, 22.5, Math.toRadians(-90));

    // Red Warehouse
    private final Pose2d posRedWarehouseApproach = new Pose2d(-1, 0, 0);
    private final Pose2d posRedWarehouseEnter = new Pose2d(-5, -36.3931, Math.toRadians(0));
    private final Pose2d posRedWarehousePark = new Pose2d(12, -36.3931, Math.toRadians(0));

    public RedWaypoints() {};

}
