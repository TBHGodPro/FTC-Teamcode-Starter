package org.firstinspires.ftc.teamcode.common.Drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface BaseDrivetrain {
    HardwareMap hardwareMap = null;

    void setPowers(double frontLeft, double frontRight, double backLeft, double backRight);

    Double getDirection();
}
