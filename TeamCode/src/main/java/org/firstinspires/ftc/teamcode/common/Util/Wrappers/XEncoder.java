package org.firstinspires.ftc.teamcode.common.Util.Wrappers;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class XEncoder<T> {
    public final Motor.Encoder encoder;

    private Motor.Direction direction = Motor.Direction.FORWARD;
    private Double distancePerPulse = 0d;

    private T lastPosition;

    public XEncoder(Motor.Encoder encoder) {
        this.encoder = encoder;
    }

    public T getPosition() {
        this.lastPosition = (T) (Integer) encoder.getPosition();

        return lastPosition;
    }

    public T getLastPosition() {
        return lastPosition;
    }

    public double getAcceleration() {
        return encoder.getAcceleration();
    }

    public double getRawVelocity() {
        return encoder.getRawVelocity();
    }


    public double getCorrectedVelocity() {
        return encoder.getCorrectedVelocity();
    }

    public double getRate() {
        return encoder.getRate();
    }

    public double getRevolutions() {
        return encoder.getRevolutions();
    }

    public double getDistance() {
        return encoder.getDistance();
    }

    public XEncoder setDirection(Motor.Direction direction) {
        this.direction = direction;

        encoder.setDirection(direction);

        return this;
    }

    public Motor.Direction getDirection() {
        return direction;
    }

    public XEncoder setDistancePerPulse(double distancePerPulse) {
        this.distancePerPulse = distancePerPulse;

        encoder.setDistancePerPulse(distancePerPulse);

        return this;
    }

    public double getDistancePerPulse() {
        return distancePerPulse;
    }

    public XEncoder reset() {
        encoder.reset();

        return this;
    }
}
