package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

@Disabled
@TeleOp(name = "LinearTeleOp", group = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "m1");

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (opModeIsActive()) {
            resetCache(allHubs);

            //getInputVoltage = returns battery voltage
            //getCurrent = current draw from the whole hub

            for(LynxModule hub : allHubs){
                double batteryVoltage = hub.getInputVoltage(VoltageUnit.VOLTS);
                double current = hub.getCurrent(CurrentUnit.AMPS);
                telemetry.addData("Battery Voltage", batteryVoltage);
                telemetry.addData("Current", current);
                telemetry.update();
            }

        }
    }

    public void resetCache(List<LynxModule> allHubs){
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }
}
