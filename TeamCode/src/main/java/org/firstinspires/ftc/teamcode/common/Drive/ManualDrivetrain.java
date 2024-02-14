package org.firstinspires.ftc.teamcode.common.Drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class ManualDrivetrain implements BaseDrivetrain {
    public final HardwareMap hardwareMap;

    public ManualDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void setPowers(double frontLeft, double frontRight, double backLeft, double backRight) {

    }

    @Override
    public Double getDirection() {
        return null;
    }
}
