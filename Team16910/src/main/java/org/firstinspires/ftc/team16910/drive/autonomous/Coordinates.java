package org.firstinspires.ftc.team16910.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Coordinates {

    // Positions as Pose2d
    public static Pose2d BlueWarehouseBarcode,BlueWarehouseHub_3, BlueWarehouseHub, BlueWarehouseEnd_3, BlueWarehouseEnd;
    public static Pose2d BlueStorageBarcode,BlueStorageCarousel, BlueStorageHub_3, BlueStorageHub, BlueStorageEnd_3, BlueStorageEnd;

    public static Pose2d RedWarehouseBarcode,RedWarehouseHub_3, RedWarehouseHub, RedWarehouseEnd_3, RedWarehouseEnd;
    public static Pose2d RedStorageBarcode, RedStorageCarousel, RedStorageHub_3, RedStorageHub, RedStorageEnd_3, RedStorageEnd;

    //public static Vector2d RedCarousel, BlueCarousel, RedHub, BlueHub, RedFinish, BlueFinish;

    public static void Positions()
    {

        BlueWarehouseBarcode = new Pose2d(27.102410493136468, 25.8890852672091, 0);
        BlueWarehouseHub_3 = new Pose2d(-20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        BlueWarehouseHub = new Pose2d(20.37210776664967, 22.922848844623836, 0);
        BlueWarehouseEnd_3 = new Pose2d(-18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        BlueWarehouseEnd = new Pose2d (18.641856780246332, 72.96895702032774, Math.toRadians(90));

        BlueStorageCarousel = new Pose2d (-5.1327873025844015, -25.261654154033103, Math.toRadians(180));
        BlueStorageBarcode = new Pose2d (4, 24.422398284891067, 0);
        BlueStorageHub_3 = new Pose2d(20.093903824570501, 23.754585428696576, 0);
        BlueStorageHub = new Pose2d(-20.093903824570501, 23.754585428696576, Math.toRadians(180));
        BlueStorageEnd_3 = new Pose2d (-26.706247391511965, -27.18861475380927, Math.toRadians(180));
        BlueStorageEnd = new Pose2d (26.706247391511965, -27.18861475380927, 0);



        RedWarehouseBarcode = new Pose2d(-27.102410493136468, 25.8890852672091, 0);
        RedWarehouseHub_3 = new Pose2d(20.37210776664967, -22.922848844623836, Math.toRadians(180)); //Third level needs robot to be different orientation then for lvl 1 and 2
        RedWarehouseHub = new Pose2d(-20.37210776664967, 22.922848844623836, 0);
        RedWarehouseEnd_3 = new Pose2d(18.641856780246332, 72.96895702032774, Math.toRadians(-90));//If we end at lvl 3
        RedWarehouseEnd = new Pose2d (-18.641856780246332, 72.96895702032774, Math.toRadians(90));

        RedStorageCarousel = new Pose2d (5.1327873025844015, -25.261654154033103, Math.toRadians(180));
        RedStorageBarcode = new Pose2d (-4, 24.422398284891067, 0);
        RedStorageHub_3 = new Pose2d(-20.093903824570501, 23.754585428696576, 0);
        RedStorageHub = new Pose2d(20.093903824570501, 23.754585428696576, Math.toRadians(180));
        RedStorageEnd_3 = new Pose2d (26.706247391511965, -27.18861475380927, Math.toRadians(180));
        RedStorageEnd = new Pose2d (-26.706247391511965, -27.18861475380927, 0);
    }
}