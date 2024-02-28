package org.firstinspires.ftc.teamcode.common.Managers;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import java.util.List;
import java.util.function.Consumer;

import javax.annotation.Nullable;

public class HubManager {
    public final List<LynxModule> hubs;
    public final LynxModule controlHub;
    @Nullable
    public LynxModule expansionHub;

    public HubManager(HardwareMap hardwareMap) throws Exception {
        this.hubs = hardwareMap.getAll(LynxModule.class);

        int numHubs = this.hubs.size();
        if (numHubs < 1 || numHubs > 2) throw new Exception("Must have either 1 or 2 hubs.");
        else if (numHubs == 1) {
            controlHub = this.hubs.get(0);
            expansionHub = null;
        } else if (hubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(hubs.get(0).getSerialNumber())) {
            controlHub = this.hubs.get(0);
            expansionHub = this.hubs.get(1);
        } else {
            controlHub = this.hubs.get(1);
            expansionHub = this.hubs.get(0);
        }
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
