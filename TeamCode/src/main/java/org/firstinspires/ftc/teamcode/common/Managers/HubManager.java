package org.firstinspires.ftc.teamcode.common.Managers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.function.Consumer;

public class HubManager {
    public final List<LynxModule> hubs;

    public HubManager(HardwareMap hardwareMap) {
        this.hubs = hardwareMap.getAll(LynxModule.class);
    }

    public void runOnEachHub(Consumer<LynxModule> func) {
        for (LynxModule hub : hubs) {
            func.accept(hub);
        }
    }

    public void useAutomaticCaching() {
        runOnEachHub(
                hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        );
    }

    public void useManualCaching() {
        runOnEachHub(
                hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        );
    }


    public void useNoCaching() {
        runOnEachHub(
                hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF)
        );
    }

    public void updateCache() {
        runOnEachHub(
                hub -> hub.clearBulkCache()
        );
    }
}
