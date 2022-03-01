package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.opencv.core.Mat;

import java.net.PortUnreachableException;

@Config
public class PoseStorage {

    // Positions as Pose2d
    public static Pose2d RedLeft, RedRight, BlueLeft, BlueRight;
    public static Pose2d RedStorageUnit,  RedCarousel, BlueCarousel,
            BlueStorageUnit, RedWarehouseSetupB, BlueWarehouseSetupB,
            RedWarehouseBlocks, BlueWarehouseBlocks;

    public static Vector2d RedHubL, RedHubR, BlueHubR, BlueHubL, RedWarehouseSetupA, BlueWarehouseSetupA;

    public static Pose2d DetStartPose(int allI, int sideI)
    {
        // Red
        if(allI == 0)
        {
            //Left
            if(sideI == 0)
            {
                return RedLeft;
            }
            return RedRight;
        }
        // Blue
        if(sideI == 0)
        {
            return BlueLeft;
        }
        return BlueRight;
    }


    public static void initPoses() {


        // Start poses
        RedLeft = new Pose2d(-36.04860248750233, -60, Math.toRadians(-90));
        RedRight = new Pose2d(12, -60, Math.toRadians(-90));
        BlueLeft = new Pose2d(13.04860248750233, 64.287627195530014, Math.toRadians(90));
        BlueRight = new Pose2d(-35.04860248750233, 64.287627195530014, Math.toRadians(90));

        // Component poses
        RedCarousel = new Pose2d(-55, -53, Math.toRadians(270));
        BlueCarousel = new Pose2d(-61.5, 52, Math.toRadians(145));

        // Shipping Hubs
        RedHubL = new Vector2d( -15,-43);
        RedHubR = new Vector2d(-9, -45);
        BlueHubL = new Vector2d( -9, 43);
        BlueHubR = new Vector2d( -15, 45);

        // Storage Units
        RedStorageUnit = new Pose2d(-60.66711740422015, -35, Math.toRadians(0));
        BlueStorageUnit = new Pose2d(-58, 34, Math.toRadians(20));

        // Warehouse poses
        RedWarehouseSetupA = new Vector2d(0, -50);
        RedWarehouseSetupB = new Pose2d(0, -64, Math.toRadians(90));
        BlueWarehouseSetupA = new Vector2d(8, 45);
        BlueWarehouseSetupB = new Pose2d(0, 67, Math.toRadians(280));

        // Warehouse blocks
        RedWarehouseBlocks = new Pose2d(57, -48, Math.toRadians(-30));
        BlueWarehouseBlocks = new Pose2d(55, 60, Math.toRadians(15));

    }

}
