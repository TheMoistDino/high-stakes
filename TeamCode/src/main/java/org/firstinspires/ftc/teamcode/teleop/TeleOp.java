package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.AdvGamepad;
import org.firstinspires.ftc.teamcode.control.CameraControl;
import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.LynxModuleControl;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.SensorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;
import org.firstinspires.ftc.teamcode.util.ColorSortManager;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode
{
    // Variables for Method Calling
    HolonomicDrive drive;
    ServoControl servo;
    MotorControl motor;
    SensorControl sensor;
    CameraControl camera;
    LynxModuleControl lynxModule;
    AdvGamepad gamepad;
    ColorSortManager colorSortManager;
    //////////////////////

    // Misc Variables
    boolean isFieldOriented = false;
    boolean colorSort = true;
    ElapsedTime runtime = new ElapsedTime(),
                clampTimer = new ElapsedTime();
    double timeout = 100, clampTimeout = 5000;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // For Holonomic Drive
        drive = new HolonomicDrive(hardwareMap, telemetry);
        //////////////////////

        // For Servo Control (Clamp)
        servo = new ServoControl(hardwareMap, telemetry);
        //////////////////////

        // For Motor Control (Lady Brown, Intake)
        motor = new MotorControl(hardwareMap, telemetry);
        //////////////////////

        // For Sensor Control (Color Sensor, Distance Sensor)
        //sensor = new SensorControl(hardwareMap, telemetry);
        //////////////////////

        // For Camera Control
        // camera = new CameraControl(hardwareMap, telemetry);
        //////////////////////

        // For Bulk Reading
        lynxModule = new LynxModuleControl(hardwareMap, telemetry);
        lynxModule.init();
        //////////////////////

        // For Gamepad State Storage
        gamepad = new AdvGamepad(gamepad1, gamepad2);
        //////////////////////

        // For Color Sort Manager
        colorSortManager = new ColorSortManager();
        //////////////////////

        // Initialize the map with lambda expressions for each action
        // Toggles
        gamepad.addAction(1, AdvGamepad.GamepadInput.back, AdvGamepad.InputType.onPress, () -> isFieldOriented = !isFieldOriented);
        gamepad.addAction(1, AdvGamepad.GamepadInput.right_bumper, AdvGamepad.InputType.onPress, () -> servo.ToggleClamp());
        gamepad.addAction(1, AdvGamepad.GamepadInput.x, AdvGamepad.InputType.onPress, () -> colorSortManager.toggleColorSort());
        gamepad.addAction(1, AdvGamepad.GamepadInput.dpad_down, AdvGamepad.InputType.onPress, () -> motor.intake(MotorControl.IntakeDirection.in, 1));
        gamepad.addAction(1, AdvGamepad.GamepadInput.dpad_up, AdvGamepad.InputType.onPress, () -> motor.intake(MotorControl.IntakeDirection.out, 1));
        gamepad.addAction(1, AdvGamepad.GamepadInput.dpad_left, AdvGamepad.InputType.onPress, () -> motor.intake(MotorControl.IntakeDirection.none, 1));
        gamepad.addAction(1, AdvGamepad.GamepadInput.dpad_right, AdvGamepad.InputType.onPress, () -> motor.intake(MotorControl.IntakeDirection.none, 1));
        gamepad.addAction(1, AdvGamepad.GamepadInput.b, AdvGamepad.InputType.onPress, () -> motor.intake(MotorControl.IntakeDirection.none, 1));
        gamepad.addAction(1, AdvGamepad.GamepadInput.left_bumper, AdvGamepad.InputType.onPress, () -> servo.ToggleDoinker());

        // Button holds
        //gamepad.addAction(1, AdvGamepad.GamepadInput.right_bumper, AdvGamepad.InputType.onButtonHold, () -> servo.OpenClamp());

        telemetry.addData("Robot Status","Ready");
        telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
        telemetry.update();

        while(opModeInInit())
        {
            // Clear the cache to avoid memory leaks and other issues (allows for bulk reading)
            lynxModule.resetCache();

            if(gamepad1.back)
            {
                isFieldOriented = !isFieldOriented;
                while(runtime.milliseconds() < timeout)
                {
                    telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
                    telemetry.update();
                }
            }

            telemetry.addData("Robot Status","Ready & Initialized");
            telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
            // telemetry.addData("Distance", sensor.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        // Wait for the play button to be pressed
        waitForStart();

        // Start servos after the play button is pressed to avoid movement between AUTO and TELEOP periods
        servo.StartServos();

        // Reset the clamp timer
        clampTimer.reset();

        while(opModeIsActive())
        {
            // Clear the cache to avoid memory leaks and other issues (allows for bulk reading)
            lynxModule.resetCache();
            //////////////////////

            // Gamepad controls are updated & executed
            gamepad.updateGamepad(gamepad1, gamepad2);
            //////////////////////

            // For Holonomic Drive
            drive.ActiveDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1.0, isFieldOriented);
            //////////////////////

            // TeleOp Assist
            //sensor.autoClamp();

//            if (colorSortManager.isRedColorSortActive()) {
//                sensor.redColorSort(true); // Assuming you have a redColorSort method
//                sensor.blueColorSort(false);
//            } else {
//                sensor.redColorSort(false);
//                sensor.blueColorSort(true); // Assuming you have a blueColorSort method
//            }

            //sensor.redColorSort(colorSort);
            //sensor.blueColorSort(colorSort);
            //camera.redColorSort(colorSort);
            //camera.blueColorSort(colorSort);
            //////////////////////

            //sensor.updateDistance();

            telemetry.addData("Robot Status","TELEOP Running");
            telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
            //telemetry.addData("Color Sort", colorSort);
            telemetry.addData("Clamp Status", servo.isClamp);
            telemetry.addData("Doinker Status", servo.isDoinkerDown);
            telemetry.addData("Intake Speed", motor.intakeMotor.getVelocity());

            telemetry.update();
        }
    }
}
