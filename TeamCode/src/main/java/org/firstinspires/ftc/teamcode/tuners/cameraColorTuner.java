package org.firstinspires.ftc.teamcode.tuners;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(name="Camera Color Tuner", group="Test")
public class cameraColorTuner extends LinearOpMode
{
    public static boolean MASK_TOGGLE = false;
    public static Scalar RANGE_LOW = new Scalar(50,50,50,0);
    public static Scalar RANGE_HIGH = new Scalar(140,140,140,255);

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource{

        Mat peopleMask = new Mat();
        private final AtomicReference<Scalar> mean = new AtomicReference<>(new Scalar(0,0,0,0));
        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));
        private final AtomicReference<Bitmap> peopleMaskedFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        public Bitmap getMaskedFrameBitmap(){
            return peopleMaskedFrame.get();
        }
        public Bitmap getLastFrame(){
            return lastFrame.get();
        }

        public Scalar getMeanColor(){
            return mean.get();
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            mean.set(Core.mean(frame));
            Core.inRange(frame, RANGE_LOW, RANGE_HIGH, peopleMask);
            Bitmap b_peopleMask = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(peopleMask, b_peopleMask);
            peopleMaskedFrame.set(b_peopleMask);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        int percentMask;
        boolean isPersonDetected;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            if (MASK_TOGGLE) {
                FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
            } else {
                FtcDashboard.getInstance().sendImage(processor.getLastFrame());
            }

            //percentMask = (int) (100 * Core.countNonZero(processor.peopleMask) / (processor.peopleMask.width() * processor.peopleMask.height()));
            //isPersonDetected = percentMask > 50;

            telemetry.addData("AverageColor: ", processor.getMeanColor().toString());
            //telemetry.addData("Percent Mask: ", percentMask);
            //telemetry.addData("Is Person Detected: ", isPersonDetected);
            telemetry.update();
            sleep(100);
        }
    }
}
