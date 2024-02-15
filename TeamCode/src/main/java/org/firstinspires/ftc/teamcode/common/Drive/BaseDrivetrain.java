package org.firstinspires.ftc.teamcode.common.Drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface BaseDrivetrain {
    HardwareMap hardwareMap = null;

    void setWheelPowers(double frontLeft, double frontRight, double backLeft, double backRight);

    Double getHeading();
}
