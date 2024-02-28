package org.firstinspires.ftc.teamcode.common.Pathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.common.Util.MathUtils;

public class Pose extends Point {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Pose pose) {
        super(pose);
        this.heading = pose.heading;
    }

    public Pose(Pose2d pose) {
        super(pose.vec());
        this.heading = MathUtils.radiansToDegrees(-pose.getHeading());
    }

    public Pose(Point point, double heading) {
        super(point);
        this.heading = heading;
    }

    public Pose(Vector2d vector, double heading) {
        super(vector);
        this.heading = heading;
    }

    public Pose set(double x, double y, double heading) {
        super.set(x, y);
        this.heading = heading;

        return this;
    }

    public Pose add(double x, double y, double heading) {
        super.add(x, y);
        this.heading += heading;

        return this;
    }

    public Pose addNew(double x, double y, double heading) {
        return new Pose(this.x + x, this.y + y, this.heading + heading);
    }

    public Pose sub(double x, double y, double heading) {
        super.sub(x, y);
        this.heading -= heading;

        return this;
    }

    public Pose subNew(double x, double y, double heading) {
        return new Pose(this.x - x, this.y - y, this.heading - heading);
    }

    public Pose mult(double x, double y, double heading) {
        super.mult(x, y);
        this.heading *= heading;

        return this;
    }

    public Pose multNew(double x, double y, double heading) {
        return new Pose(this.x * x, this.y * y, this.heading * heading);
    }

    public Pose div(double x, double y, double heading) {
        super.div(x, y);
        this.heading /= heading;

        return this;
    }

    public Pose divNew(double x, double y, double heading) {
        return new Pose(this.x / x, this.y / y, this.heading / heading);
    }

    public Pose2d toPose2d() {
        return new Pose2d(this.toVector2d(), MathUtils.degreesToRadians(-this.heading));
    }

    public static Pose fromPose2d(Pose2d pose) {
        return new Pose(pose);
    }

    public Pose2d p() {
        return this.toPose2d();
    }
}
