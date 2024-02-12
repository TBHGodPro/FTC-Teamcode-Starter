package org.firstinspires.ftc.teamcode.common.Util.Wrappers;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class XIMU {
    public final IMU imu;

    public final AngleUnit preferredUnit;

    public double lastYaw = 0;
    public double lastPitch = 0;
    public double lastRoll = 0;
    public long lastAcquisitionTime = 0;

    public AngularVelocity lastVelocity;

    public XIMU(IMU imu, IMU.Parameters params, AngleUnit preferredUnit) {
        this.imu = imu;

        imu.initialize(params);

        this.preferredUnit = preferredUnit;

        this.lastVelocity = new AngularVelocity(preferredUnit, 0, 0, 0, 0);

        resetYaw();
    }

    public double getYawDegrees() {
        return getYawPitchRoll().getYaw(AngleUnit.DEGREES);
    }

    public double getYawRadians() {
        return getYawPitchRoll().getYaw(AngleUnit.RADIANS);
    }

    public double getPitchDegrees() {
        return getYawPitchRoll().getPitch(AngleUnit.DEGREES);
    }

    public double getPitchRadians() {
        return getYawPitchRoll().getPitch(AngleUnit.RADIANS);
    }

    public double getRollDegrees() {
        return getYawPitchRoll().getRoll(AngleUnit.DEGREES);
    }

    public double getRollRadians() {
        return getYawPitchRoll().getRoll(AngleUnit.RADIANS);
    }

    public double getYaw() {
        getYawPitchRoll();

        return lastYaw;
    }

    public double getPitch() {
        getYawPitchRoll();

        return lastPitch;
    }

    public double getRoll() {
        getYawPitchRoll();

        return lastRoll;
    }

    public long getAcquisitionTime() {
        getYawPitchRoll();

        return lastAcquisitionTime;
    }

    public YawPitchRollAngles getYawPitchRoll() {
        YawPitchRollAngles yawPitchRoll = imu.getRobotYawPitchRollAngles();

        lastYaw = yawPitchRoll.getYaw(preferredUnit);
        lastPitch = yawPitchRoll.getPitch(preferredUnit);
        lastRoll = yawPitchRoll.getRoll(preferredUnit);
        lastAcquisitionTime = yawPitchRoll.getAcquisitionTime();

        return yawPitchRoll;
    }

    public AngularVelocity getVelocityDegrees() {
        if (preferredUnit == AngleUnit.DEGREES) {
            lastVelocity = imu.getRobotAngularVelocity(preferredUnit);

            return lastVelocity;
        } else
            return imu.getRobotAngularVelocity(AngleUnit.DEGREES);
    }

    public AngularVelocity getVelocityRadians() {
        if (preferredUnit == AngleUnit.RADIANS) {
            lastVelocity = imu.getRobotAngularVelocity(preferredUnit);

            return lastVelocity;
        } else
            return imu.getRobotAngularVelocity(AngleUnit.RADIANS);
    }

    public XIMU resetYaw() {
        imu.resetYaw();

        return this;
    }
}
