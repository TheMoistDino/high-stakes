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
        // Instantiate LynxModule Objects
        LynxModuleControl.allHubs = hardwareMap.getAll(LynxModule.class);
        // Instantiate Telemetry
        LynxModuleControl.telemetry = telemetry;

        // Display Message on Screen
        telemetry.addData("LynxModule Status", "Getting Ready...");
    }

    public void init(){
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry.addData("LynxModule Status", "Initialized");
    }

    public void resetCache()
    {
        for(LynxModule hub : allHubs)
        {
            hub.clearBulkCache();
        }
    }
}
