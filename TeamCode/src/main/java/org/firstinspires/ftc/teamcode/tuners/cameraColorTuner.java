package org.firstinspires.ftc.teamcode.tuners;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@Config
@TeleOp(name = "Camera Color Tuner", group = "Test")
public class cameraColorTuner extends LinearOpMode
{
    public static double roi = 0.1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-roi, roi, roi, -roi))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        while(opModeIsActive() || opModeInInit())
        {
            PredominantColorProcessor.Result result = colorSensor.getAnalysis();
            FtcDashboard.getInstance().startCameraStream(portal, 20);

            // Display the Color Sensor result.
            telemetry.addData("Best Match:", result.closestSwatch);
            telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));

            if(result.closestSwatch == PredominantColorProcessor.Swatch.RED)
            {
                telemetry.addData("Color Sort", "RED");
            }
            else if(result.closestSwatch == PredominantColorProcessor.Swatch.BLUE)
            {
                telemetry.addData("Color Sort", "BLUE");
            }
            else
            {
                telemetry.addData("Color Sort", "NONE");
            }

            telemetry.update();

            sleep(20);
        }
    }
}
