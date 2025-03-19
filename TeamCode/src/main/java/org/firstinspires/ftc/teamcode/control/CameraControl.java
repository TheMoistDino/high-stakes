package org.firstinspires.ftc.teamcode.control;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class CameraControl
{
    ///// Create Camera Variables
    public static PredominantColorProcessor colorSensor;
    public static VisionPortal portal;
    public static PredominantColorProcessor.Result result;
    /////

    ///// Extra variables
    static Telemetry telemetry;
    /////

    public CameraControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Camera Objects
        CameraControl.colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.15, 0.15, 0.15, -0.15))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        CameraControl.portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // Instantiate Telemetry
        CameraControl.telemetry = telemetry;

        // Display Message on Screen
        telemetry.addData("Camera Status", "Initialized");
    }

    public void updateColor()
    {
        result = colorSensor.getAnalysis();

        // Display the Color Sensor result.
        telemetry.addData("Best Match", result.closestSwatch);
        telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
    }

    public void redColorSort(boolean colorSort)
    {
        updateColor();
        if((result.closestSwatch == PredominantColorProcessor.Swatch.RED) && colorSort)
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
        if((result.closestSwatch == PredominantColorProcessor.Swatch.BLUE) && colorSort)
        {
            telemetry.addData("Color Sort", "BLUE");
            MotorControl.reject();
        }
        else
        {
            telemetry.addData("Color Sort", "NONE");
        }
    }

}
