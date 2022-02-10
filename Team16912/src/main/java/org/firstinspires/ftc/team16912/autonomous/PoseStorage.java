package org.firstinspires.ftc.team16912.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.net.PortUnreachableException;

@Config
public class PoseStorage {

    // Positions as Pose2d
    public static Pose2d RedLeft, RedRight, BlueLeft, BlueRight;
    public static Pose2d BlueCarousel, BlueHub, RedStorageUnit,
            BlueStorageUnit, RedWarehouse, BlueWarehouse, RedWarehouseSetup, BlueWarehouseSetup,
            BluePickup, RedPickup,
            RedWarehouseBlocks, BlueWarehouseBlocks;

    public static Vector2d RedHub, RedCarousel;

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
        RedCarousel = new Vector2d(-55.72295700521856,-53.766810012697775);
        BlueCarousel = new Pose2d(-59.43131663678813, 56.86792328414704,Math.toRadians(172.84226364522823));
        RedHub = new Vector2d( -12,-46);
        BlueHub = new Pose2d( -10.638173890493668, 54.95355503292278, Math.toRadians(90));
        RedStorageUnit = new Pose2d(-60.66711740422015, -36.326774389878032, Math.toRadians(0));
        BlueStorageUnit = new Pose2d(-58.664135706521755, 37.818617399773153, Math.toRadians(5));

        // Warehouse poses
        RedWarehouseSetup = new Pose2d(0, -60, Math.toRadians(90));
        RedWarehouse = new Pose2d();
        BlueWarehouseSetup = new Pose2d(-10.638173890493668, 69, Math.toRadians(280));
        BlueWarehouse = new Pose2d();
        BluePickup = new Pose2d();
        RedPickup = new Pose2d();
        RedWarehouseBlocks = new Pose2d(48, -48, Math.toRadians(-45));
        BlueWarehouseBlocks = new Pose2d(55, 60, Math.toRadians(15));

    }

}
