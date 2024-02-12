package org.firstinspires.ftc.teamcode.common.Util.Wrappers;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class XIMU {
    public final IMU imu;

    public final XIMUThread thread;

    public final AngleUnit preferredUnit;

    public double lastYaw = 0;
    public double lastPitch = 0;
    public double lastRoll = 0;
    public long lastAcquisitionTime = 0;

    public AngularVelocity lastVelocity;

    public XIMU(IMU imu, IMU.Parameters params, AngleUnit preferredUnit) {
        this.imu = imu;

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

    public AngularVelocity getVelocityDegrees() {
        if (preferredUnit == AngleUnit.DEGREES) {
            lastVelocity = thread.velocityDegrees;

            return lastVelocity;
        } else return thread.velocityDegrees;
    }

    public AngularVelocity getVelocityRadians() {
        if (preferredUnit == AngleUnit.RADIANS) {
            lastVelocity = thread.velocityRadians;

            return lastVelocity;
        } else return thread.velocityRadians;
    }

    public XIMU resetYaw() {
        imu.resetYaw();

        return this;
    }

    public void stop() {
        thread.deactivate();
        thread.interrupt();
        thread.destroy();
        imu.close();
    }


}

class XIMUThread extends Thread {
    private final IMU imu;

    private boolean isActive;

    public volatile YawPitchRollAngles angles = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);
    public volatile AngularVelocity velocityDegrees = new AngularVelocity(AngleUnit.DEGREES, 0, 0, 0, 0);
    public volatile AngularVelocity velocityRadians = new AngularVelocity(AngleUnit.RADIANS, 0, 0, 0, 0);

    public XIMUThread(IMU imu) {
        this.imu = imu;

        this.isActive = false;
    }

    @Override
    public void run() {
        while (true) {
            if (isActive) {
                angles = imu.getRobotYawPitchRollAngles();
                velocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                velocityRadians = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            } else {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    deactivate();
                    interrupt();
                    break;
                }
            }
        }
    }

    public void activate() {
        this.isActive = true;
    }

    public void deactivate() {
        this.isActive = false;
    }
}
