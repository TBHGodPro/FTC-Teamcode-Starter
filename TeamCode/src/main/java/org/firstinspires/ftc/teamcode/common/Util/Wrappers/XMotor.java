package org.firstinspires.ftc.teamcode.common.Util.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Util.MotionHandler;

public class XMotor {
    public final HardwareMap hardwareMap;
    public final String name;
    public final DcMotorEx motor;
    public final VoltageSensor voltage;
    public final MotionHandler motion;

    private double manualPower = 0;

    private int offset = 0;

    private boolean shouldHandleVoltage = false;

    public XMotor(HardwareMap hardwareMap, String name, MotionHandler motion) {
        this.hardwareMap = hardwareMap;
        this.name = name;

        this.motor = hardwareMap.get(DcMotorEx.class, name);
        this.voltage = hardwareMap.voltageSensor.get(name);

        this.motion = motion;

        getPosition();
        setTargetPosition(getPosition());
    }

    public XMotor setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);

        return this;
    }

    public DcMotorEx.ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    public XMotor setShouldHandleVoltage(boolean shouldHandleVoltage) {
        this.shouldHandleVoltage = shouldHandleVoltage;

        return this;
    }

    public boolean getShouldHandleVoltage() {
        return this.shouldHandleVoltage;
    }

    public XMotor setInverted(boolean isInverted) {
        motor.setDirection(isInverted ? DcMotorEx.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        return this;
    }

    public boolean getInverted() {
        return motor.getDirection() == DcMotorSimple.Direction.REVERSE;
    }

    public XMotor setOffset(int offset) {
        this.offset = offset;

        return this;
    }

    public int getOffset() {
        return this.offset;
    }

    public double getPower() {
        return manualPower;
    }

    public int getPosition() {
        return motor.getCurrentPosition() - offset;
    }

    public double getTruePower() {
        double power = 0;

        if (manualPower == 0) {
            power += motion.calculate(getPosition());
        } else {
            power += manualPower;
            power += motion.getFeedforward();
        }

        if (voltage != null && shouldHandleVoltage)
            power *= Constants.NOMINAL_VOLTAGE / voltage.getVoltage();

        return Range.clip(power, -1, 1);
    }

    public XMotor update() {
        motor.setPower(getTruePower());

        return this;
    }

    public XMotor setTargetPosition(int position) {
        manualPower = 0;

        motion.setTarget(getPosition(), position);

        return this;
    }

    public XMotor setPower(double power) {
        manualPower = power;
        if (manualPower == 0) motion.setTarget(getPosition(), getPosition(), true);

        return this;
    }
}
