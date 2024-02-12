package org.firstinspires.ftc.teamcode.common.Util.Wrappers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class XServo {
    protected final Servo primaryServo;
    public final Servo[] servos;

    private double position;

    private boolean isInverted;

    private double maxPosition;
    private double minPosition;

    private double offset;

    /**
     * Custom Servo class to allow improved and customized usage of one or more servo.
     *
     * @param servos The built-in Servo class(es) (note that only the first one is used for encoder values and such)
     */
    public XServo(Servo... servos) {
        this.primaryServo = servos[0];
        this.servos = servos;

        this.position = 0;

        this.isInverted = false;

        this.maxPosition = 1;
        this.minPosition = 0;

        this.offset = 0;
    }

    /**
     * Invert all position reads and writes
     *
     * @param inverted Whether it should be inverted or not.
     */
    public XServo setInverted(boolean inverted) {
        this.isInverted = inverted;

        return this;
    }

    /**
     * Retrieve if all position reads and writes are inverted.
     */
    public boolean getInverted() {
        return isInverted;
    }

    /**
     * The maximum limit the Servo should be allowed to go to. Inputs will automatically clip to this position.
     *
     * @param maxPosition The Maximum Limit
     */
    public XServo setMaxPosition(double maxPosition) {
        if (maxPosition > 1) maxPosition = 1;
        if (maxPosition < 0) maxPosition = 0;

        this.maxPosition = maxPosition;

        return this;
    }

    /**
     * The minimum limit the Servo should be allowed to go to. Inputs will automatically clip to this position.
     *
     * @param minPosition The Minimum Limit
     */
    public XServo setMinPosition(double minPosition) {
        if (minPosition > 1) minPosition = 1;
        if (minPosition < 0) minPosition = 0;

        this.minPosition = minPosition;

        return this;
    }

    /**
     * Retrieve the maximum input allowed for the Servo, which inputs are automatically clipped to.
     */
    public double getMaxPosition() {
        return maxPosition;
    }

    /**
     * Retrieve the minimum input allowed for the Servo, which inputs are automatically clipped to.
     */
    public double getMinPosition() {
        return minPosition;
    }

    /**
     * Set the maximum and minimum positions the Servo is allowed to go to. Inputs will automatically clip to these limits.
     *
     * @param minPosition The Minimum Position
     * @param maxPosition The Maximum Position
     */
    public XServo setRange(double minPosition, double maxPosition) {
        setMaxPosition(maxPosition);
        setMinPosition(minPosition);

        return this;
    }

    /**
     * Set the offset of the inputted values to the Servo. This offset will work the same whether the Servo is inverted or not.
     *
     * @param offset The Numerical Offset
     */
    public XServo setOffset(double offset) {
        this.offset = offset;

        return this;
    }

    /**
     * Retrieve the offset of the Servo.
     */
    public double getOffset() {
        return this.offset;
    }

    /**
     * Set the target position of the servo. This position will automatically clip to the allowed range, account for offset, and invert if necessary.
     *
     * @param position The target position
     */
    public XServo setPosition(double position) {
        this.position = Range.clip(position + offset, minPosition, maxPosition);

        double realPos = isInverted ? (1 - this.position) : this.position;

        for (Servo servo : servos) {
            servo.setPosition(realPos);
        }

        return this;
    }

    /**
     * Retrieve the position of the servo. Will invert and account for offset if necessary.
     * <p>
     * Note that this method may return a value above or below the allowed limit if the servo includes a built-in encoder.
     */
    public double getPosition() {
        if (isInverted) this.position = (1 - primaryServo.getPosition()) + offset;
        else this.position = primaryServo.getPosition() + offset;

        return this.position;
    }

    /**
     * Disable the servo.
     */
    public XServo disable() {
        for (Servo servo : servos) {
            servo.close();
        }

        return this;
    }
}
