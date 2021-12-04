package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.net.PortUnreachableException;

public class PoseStorage {

    // Positions as Pose2d
    public static Pose2d redLeft, redRight, blueLeft, blueRight;
    public static Pose2d carousel, shipmentHub, finish;

    public static void initPoses() {


        // Start poses
        redLeft = new Pose2d();
        redRight = new Pose2d();
        blueLeft = new Pose2d();
        blueRight = new Pose2d();


        // Component poses
        carousel = new Pose2d();
        shipmentHub = new Pose2d();
        finish = new Pose2d();

    }

}
