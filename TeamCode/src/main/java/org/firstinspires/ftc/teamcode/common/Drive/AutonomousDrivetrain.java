package org.firstinspires.ftc.teamcode.common.Drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.Pathing.drive.SampleMecanumDrive;

public class AutonomousDrivetrain extends SampleMecanumDrive implements BaseDrivetrain {
    public final HardwareMap hardwareMap;

    public AutonomousDrivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);

        this.hardwareMap = hardwareMap;
    }

    @Override
    public void setPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        super.setMotorPowers(frontLeft, backLeft, backRight, frontRight);
    }

    @Override
    public Double getDirection() {
        return super.getRawExternalHeading();
    }
}
