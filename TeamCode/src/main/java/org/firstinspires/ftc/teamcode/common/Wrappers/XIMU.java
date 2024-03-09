package org.firstinspires.ftc.teamcode.common.Wrappers;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import javax.annotation.concurrent.GuardedBy;

public class XIMU {
    public final XIMUThread thread;

    public final AngleUnit preferredUnit;

    public double lastYaw = 0;
    public double lastPitch = 0;
    public double lastRoll = 0;
    public long lastAcquisitionTime = 0;

    public AngularVelocity lastVelocity;

    public XIMU(IMU imu, IMU.Parameters params, AngleUnit preferredUnit) {
        this.thread = new XIMUThread(imu);

        imu.initialize(params);

        this.preferredUnit = preferredUnit;

        this.lastVelocity = new AngularVelocity(preferredUnit, 0, 0, 0, 0);

        resetYaw();

        thread.start();
        thread.activate();
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
        YawPitchRollAngles yawPitchRoll = thread.angles;

        lastYaw = yawPitchRoll.getYaw(preferredUnit);
        lastPitch = yawPitchRoll.getPitch(preferredUnit);
        lastRoll = yawPitchRoll.getRoll(preferredUnit);
        lastAcquisitionTime = yawPitchRoll.getAcquisitionTime();

        return yawPitchRoll;
    }

    public AngularVelocity getVelocity() {
        lastVelocity = thread.velocity.toAngleUnit(preferredUnit);

        return lastVelocity;
    }

    public AngularVelocity getVelocityDegrees() {
        return getVelocity().toAngleUnit(AngleUnit.DEGREES);
    }

    public AngularVelocity getVelocityRadians() {
        return getVelocity().toAngleUnit(AngleUnit.RADIANS);
    }

    public XIMU resetYaw() {
        thread.resetYaw();

        return this;
    }

    public void stop() {
        thread.end();
    }
}

class XIMUThread extends Thread {
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private final IMU imu;

    private boolean isActive;

    public volatile YawPitchRollAngles angles = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    public volatile AngularVelocity velocity = new AngularVelocity(AngleUnit.DEGREES, 0, 0, 0, 0);

    public XIMUThread(IMU imu) {
        this.imu = imu;

        this.isActive = false;
    }

    public void end() {
        this.deactivate();
        this.interrupt();
        this.destroy();
    }

    public void resetYaw() {
        synchronized (imuLock) {
            imu.resetYaw();
        }
    }

    @Override
    public void run() {
        while (!isInterrupted()) {
            synchronized (imuLock) {
                if (isActive) {
                    angles = imu.getRobotYawPitchRollAngles();
                    velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                }
            }
        }

        synchronized (imuLock) {
            imu.close();
        }
    }

    public void activate() {
        this.isActive = true;
    }

    public void deactivate() {
        this.isActive = false;
    }
}
