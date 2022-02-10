package org.firstinspires.ftc.team16912.autonomous;

import org.opencv.core.Point;
import org.opencv.core.Rect;

public class Block
{
    public Rect rect;
    public double dist;
    public Point center;
    public double area;

    public Block(Rect rect, double dist, double area)
    {
        this.rect = rect;
        this.dist = dist;
        this.center = new Point(rect.x+(rect.width/2.0), rect.y+(rect.height/2.0));
        this.area = area;
    }

    public Block()
    {
        this.rect = new Rect(0, 0, 0, 0);
        this.dist = Double.MAX_VALUE;
        this.center = new Point(0, 0);
        this.area = Double.MAX_VALUE;
    }
}
