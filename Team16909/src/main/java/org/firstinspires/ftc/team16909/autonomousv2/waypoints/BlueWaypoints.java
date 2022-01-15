package org.firstinspires.ftc.team16909.autonomousv2.waypoints;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BlueWaypoints {
    // Blue Shipping Hub
    public final Pose2d bluePosHub = new Pose2d(19.3338, -18.4224, Math.toRadians(0));

    //Blue Carousel
    public final Pose2d posPushback = new Pose2d(4.7063, 0,Math.toRadians(0));
    public final Pose2d posBlueCarousel = new Pose2d(4.1820, -18.9322, Math.toRadians(188));
    public final Pose2d posBlueUnitPark = new Pose2d(32, -22.5, Math.toRadians(90));

    // Blue Warehouse
    public  final Pose2d posBlueWarehouseApproach = new Pose2d(-1, 0, 0);
    public  final Pose2d posBlueWarehouseEnter = new Pose2d(-2, 38.3931, Math.toRadians(0));
    public  final Pose2d posBlueWarehousePark = new Pose2d(12, 38.3931, Math.toRadians(0));

    public BlueWaypoints() {};

}
