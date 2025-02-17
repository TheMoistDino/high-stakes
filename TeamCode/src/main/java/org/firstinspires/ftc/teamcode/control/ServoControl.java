package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoControl
{
    ///// Create Servo Variables
    public static ServoImplEx clamp;
    /////

    ///// Name of Servos on Driver Hub
    private static final String clampName = "clamp";
    /////

    ///// Create and Define Motion Variables
    public static boolean isClamp;
    static final double clampOpen = 0.35,
                        clampClose = 0.50;
    /////

    ///// Create and Define Timer Variables to let the servos have time to run to position
    private final ElapsedTime runtime = new ElapsedTime();
    double timeout = 250;
    /////

    ///// Extra variables
    static Telemetry telemetry;
    /////

    // This method is used to initialize the servos of the robot
    public ServoControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Servo Objects
        ServoControl.clamp = hardwareMap.get(ServoImplEx.class, clampName);
        // Instantiate Telemetry
        ServoControl.telemetry = telemetry;

        // Increase max range of servos
        clamp.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Disable servo power by default
        StopServos();

        // Display Message on Screen
        telemetry.addData("Servo Status", "Initialized");
    }

    // This method is used to "turn on" the servos
    public void StartServos()
    {
        // Enable servo power
        clamp.setPwmEnable();

        // Start the servos in initial position
        clamp.setPosition(clampOpen);
        isClamp = false;

        // Display Message on Screen
        telemetry.addData("Servo Status", "Started");
    }

    // This method is used to "turn off" the servos
    public void StopServos()
    {
        // Disable servo power
        clamp.setPwmDisable();

        // Display Message on Screen
        telemetry.addData("Servo Status", "Stopped");
    }

    // This method is used to toggle the clamp
    public void ToggleClamp()
    {
        // Restart timer
        runtime.reset();

        // Open/close clamp
        isClamp = !isClamp;
        clamp.setPosition(isClamp ? clampClose : clampOpen);

        // Give time for the servo to run to position
        while(runtime.milliseconds() < timeout)
        {
            telemetry.addData("Clamp Status", "Running");
            telemetry.update();
        }

        // Display Message on Screen
        telemetry.addData("Clamp Status", isClamp ? "Open" : "Closed");
        telemetry.update();
    }
}