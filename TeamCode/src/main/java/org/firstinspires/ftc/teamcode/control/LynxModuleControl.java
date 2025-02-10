package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class LynxModuleControl
{
    ///// Create LynxModuleControl / Hub Variables
    public static List<LynxModule> allHubs;
    static Telemetry telemetry;

    public LynxModuleControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        LynxModuleControl.allHubs = hardwareMap.getAll(LynxModule.class);
        LynxModuleControl.telemetry = telemetry;
    }

    public void init(){
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void resetCache()
    {
        for(LynxModule hub : allHubs)
        {
            hub.clearBulkCache();
        }
    }
}
