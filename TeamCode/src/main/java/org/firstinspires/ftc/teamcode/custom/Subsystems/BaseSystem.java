package org.firstinspires.ftc.teamcode.custom.Subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public abstract class BaseSystem {
    public final HardwareMap hardwareMap;

    public BaseSystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void addTelemetry(MultipleTelemetry telemetry) {
    }
}
