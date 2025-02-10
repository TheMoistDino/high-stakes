package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Servo Tuner", group = "Test")
public class servoTuner extends OpMode {
    public static double target_servo = 0.0;

    ServoImplEx servo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = hardwareMap.get(ServoImplEx.class, "servo");
    
        servo.setPwmRange(new PwmControl.PwmRange(500,2500));
    }

    @Override
    public void loop() {
        // Sets servo position
        servo.setPosition(target_servo);

        // Adds data to the telemetry/driver hub
        telemetry.addData("servo target", target_servo);
        telemetry.update();
    }
}
