package org.firstinspires.ftc.teamcode.common.Drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Pathing.drive.MecanumDrive;

public class AutonomousDrivetrain extends MecanumDrive implements BaseDrivetrain {
    public final HardwareMap hardwareMap;

    public AutonomousDrivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);

        this.hardwareMap = hardwareMap;
    }

    @Override
    public void setWheelPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        super.setMotorPowers(frontLeft, backLeft, backRight, frontRight);
    }

    @Override
    public Double getHeading() {
        return super.getRawExternalHeading();
    }

    @Override
    public void stop() {
        super.stop();
    }

    public void update() {
        super.update();
    }
}
