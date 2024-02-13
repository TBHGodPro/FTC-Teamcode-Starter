package org.firstinspires.ftc.teamcode.common.Pathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        set(x, y);
    }

    public Point(Point point) {
        set(point.x, point.y);
    }

    public Point(Vector2d vector) {
        set(-vector.getY(), vector.getX());
    }

    public Point set(double x, double y) {
        this.x = x;
        this.y = y;

        return this;
    }

    public Point add(double x, double y) {
        this.x += x;
        this.y += y;

        return this;
    }

    public Point addNew(double x, double y) {
        return new Point(this.x + x, this.y + y);
    }

    public Point sub(double x, double y) {
        this.x -= x;
        this.y -= y;

        return this;
    }

    public Point subNew(double x, double y) {
        return new Point(this.x - x, this.y - y);
    }

    public Point mult(double x, double y) {
        this.x *= x;
        this.y *= y;

        return this;
    }

    public Point multNew(double x, double y) {
        return new Point(this.x * x, this.y * y);
    }

    public Point div(double x, double y) {
        this.x /= x;
        this.y /= y;

        return this;
    }

    public Point divNew(double x, double y) {
        return new Point(this.x / x, this.y / y);
    }

    public Vector2d toVector2d() {
        return new Vector2d(this.y, -this.x);
    }

    public static Point fromVector2d(Vector2d vector) {
        return new Point(vector);
    }

    public Vector2d v() {
        return this.toVector2d();
    }

    public Pose2d p() {
        return new Pose(this, 0).p();
    }
}
