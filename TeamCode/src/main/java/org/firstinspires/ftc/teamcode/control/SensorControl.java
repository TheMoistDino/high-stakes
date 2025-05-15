package org.firstinspires.ftc.teamcode.control;

import static org.firstinspires.ftc.teamcode.control.ServoControl.clamp;
import static org.firstinspires.ftc.teamcode.control.ServoControl.clampClose;
import static org.firstinspires.ftc.teamcode.control.ServoControl.isClamp;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorControl
{
    ///// Create Sensor Variables
    public static ColorSensor colorSensor1, colorSensor2;
    public static Rev2mDistanceSensor distanceSensor;
    /////

    ///// Name of Sensors on Driver Hub
    private static final String colorSensor1Name = "colorSensor1",
                                colorSensor2Name = "colorSensor2";
    private static final String distanceSensorName = "distance";
    /////

    ///// Create Color Variables
    public static int[] color1 = new int[3], color2 = new int[3]; // [red, green, blue]
    static double color_threshold = 20;
    /////

    ///// Distance Variables
    private final double maxDistance = 2.0;
    /////

    ///// Create and Define Timer Variables
    private final ElapsedTime clampTimer = new ElapsedTime(), distanceTimer = new ElapsedTime();
    private boolean lastClampState;
    private final double clampDelay = 125;
    /////

    ///// Extra variables
    static Telemetry telemetry;
    /////

    public SensorControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Sensor Objects
        SensorControl.colorSensor1 = hardwareMap.get(ColorSensor.class, colorSensor1Name);
        SensorControl.colorSensor2 = hardwareMap.get(ColorSensor.class, colorSensor2Name);
        SensorControl.distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, distanceSensorName);
        // Instantiate Telemetry
        SensorControl.telemetry = telemetry;

        // Start Timers
        clampTimer.reset();
        distanceTimer.reset();

        // Display Message on Screen
        telemetry.addData("Sensor Status", "Initialized");
    }

    public void updateDistance()
    {
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
    }

    public boolean checkDistance()
    {
        return (distanceSensor.getDistance(DistanceUnit.CM) < maxDistance);
    }

    public void updateColor()
    {
        color1[0] = colorSensor1.red();
        color1[1] = colorSensor1.green();
        color1[2] = colorSensor1.blue();
        color2[0] = colorSensor2.red();
        color2[1] = colorSensor2.green();
        color2[2] = colorSensor2.blue();
    }

    public void autoClamp()
    {
        if(isClamp != lastClampState)
        {
            clampTimer.reset();
        }

        if((distanceSensor.getDistance(DistanceUnit.CM) < 10) && ((clampTimer.seconds() >= 1) && (!isClamp)))
        {
            // Reset Timer
            distanceTimer.reset();

            // Clamp Delay
            while(distanceTimer.milliseconds() < clampDelay)
            {
                telemetry.addData("Clamp Status", "Mobile Goal Detected");
            }

            // Close clamp
            clamp.setPosition(clampClose);
            isClamp = true;

            // Display Message On Screen
            telemetry.addData("Clamp Status", "AUTO CLAMP");
        }
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.CM));
        lastClampState = isClamp;
    }

    public void redColorSort(boolean colorSort)
    {
        updateColor();
        if(isRedDetected(color1) && isRedDetected(color2) && colorSort)  // Check if color 1 & 2 are red
        {
            telemetry.addData("Color Sort", "RED");
            MotorControl.reject();
        }
        else
        {
            telemetry.addData("Color Sort", "NONE");
        }
    }

    public void blueColorSort(boolean colorSort)
    {
        updateColor();
        if(isBlueDetected(color1) && isBlueDetected(color2) && colorSort)  // Check if color 1 & 2 are blue
        {
            telemetry.addData("Color Sort", "BLUE");
            MotorControl.reject();
        }
        else
        {
            telemetry.addData("Color Sort", "NONE");
        }
    }

    // Helper method to check if a color is red
    private boolean isRedDetected(int[] color) {
        return color[0] > (color[1] - color_threshold) && color[0] > (color[2] - color_threshold);
    }

    // Helper method to check if a color is blue
    private boolean isBlueDetected(int[] color) {
        return color[2] > (color[0] - color_threshold) && color[2] > (color[1] - color_threshold);
    }

}
