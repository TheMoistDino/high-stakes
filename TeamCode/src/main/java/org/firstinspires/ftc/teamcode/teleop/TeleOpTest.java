package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.a;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.b;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.back;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.dpad_down;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.dpad_left;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.dpad_right;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.left_bumper;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.left_trigger;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.right_bumper;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.right_trigger;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.GamepadInput.start;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.InputType.onButtonHold;
import static org.firstinspires.ftc.teamcode.control.AdvGamepad.InputType.onPress;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.control.AdvGamepad;
import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;

@Disabled
@TeleOp(name = "Test (OLD)", group = "TeleOp")
public class TeleOpTest extends LinearOpMode
{
    // Variables for Method Calling
    HolonomicDrive drive;
    //TankDrive tankDrive;
    ServoControl servo;
    MotorControl motor;
    AdvGamepad gamepad;
    //////////////////////

    double DRIVETRAIN_SPEED_MULTIPLIER = 1.0;
    boolean driveSlow = false;
    double LIFT_SPEED_MULTIPLIER = 1.0;
    boolean liftSlow = false;
    double ARM_SPEED_MULTIPLIER = 1.0;
    boolean armSlow = false;

    boolean isFieldOriented = false; // Initialize a variable to store the current driving mode
    boolean liftDebug = false; // Initialize a variable to store the current lift debug mode

    @Override
    public void runOpMode() throws InterruptedException
    {
        // For Holonomic Drive
        drive = new HolonomicDrive(hardwareMap, telemetry);
        //////////////////////

        // For Tank Drive
        //tankDrive = new TankDrive(hardwareMap, telemetry);
        //////////////////////

        // For Servo Control (Claws)
        servo = new ServoControl(hardwareMap, telemetry);
        //////////////////////

        // For Motor Control (Lift)
        motor = new MotorControl(hardwareMap, telemetry);
        //////////////////////

        // For gamepad state storage
        gamepad = new AdvGamepad(gamepad1, gamepad2);
        //////////////////////

        // Initialize the map with lambda expressions for each action
        // Toggles
        gamepad.addAction(1, start, onPress, () -> isFieldOriented = !isFieldOriented);
        gamepad.addAction(1, left_bumper, onPress, () -> driveSlow = !driveSlow);
        gamepad.addAction(2, start, onPress, () -> liftSlow = !liftSlow);
        gamepad.addAction(2, b, onPress, () -> liftDebug = !liftDebug);
        gamepad.addAction(2, back, onPress, () -> {
            MotorControl.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorControl.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });

        // Presets
        gamepad.addAction(2, dpad_right, onPress, () -> {
            //motor.LiftToPosition(idle, 4.5);
        });
        gamepad.addAction(2, dpad_left, onPress, () -> {
            //motor.LiftToPosition(zero, 4.5);
        });

        // Servo Control
        gamepad.addAction(2, right_bumper, onPress, () -> {
            //servo.GrabOuttake();
        });
        gamepad.addAction(2, left_bumper, onPress, () -> {
            //servo.GrabIntake();
        });

        // Motor Control
        // Buttons to move arm forward/backward
        gamepad.addAction(2, a, onButtonHold, () -> {
            //motor.ArmControl(forward, ARM_SPEED_MULTIPLIER);
        });
        gamepad.addAction(2, dpad_down, onButtonHold, () -> {
            //motor.ArmControl(backward, ARM_SPEED_MULTIPLIER);
        });
        // Buttons to move lift up/down
        gamepad.addAction(2, right_trigger, onButtonHold, (liftDebug || motor.currentLiftPos < motor.maxLiftPos), () -> {
            //motor.MoveLift(up, LIFT_SPEED_MULTIPLIER);
        });
        gamepad.addAction(2, left_trigger, onButtonHold, (liftDebug || motor.currentLiftPos > motor.minLiftPos), () -> {
            //motor.MoveLift(down, LIFT_SPEED_MULTIPLIER);
        });
        //////////////////////

        telemetry.addData("Robot Status","Ready");
        telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        // Start servos after the play button is pressed to avoid movement between AUTO and TELEOP periods
        servo.StartServos();

        while(opModeIsActive())
        {
            // Gamepad controls are updated & executed
            gamepad.updateGamepad(gamepad1, gamepad2);

            // For Holonomic Drive
            drive.ActiveDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED_MULTIPLIER, isFieldOriented);
            //////////////////////

            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
            {
                //motor.LockLift(); // Brake lift when no direction is specified
            }

            // If the lift is beyond the set min or max positions, return the lift to the nearest limit
            if ((motor.currentLiftPos < motor.minLiftPos || motor.currentLiftPos > motor.maxLiftPos) && (!liftDebug))
            {
                //motor.StopAndReturnLift();
            }


            ////////////////////////////////////
            // Toggle to slow down driving
            DRIVETRAIN_SPEED_MULTIPLIER = driveSlow ? 0.4 : 0.8;
            // Toggle to slow down lift
            LIFT_SPEED_MULTIPLIER = liftSlow ? 0.5 : 1.0;
            // Toggle to slow down arm
            ARM_SPEED_MULTIPLIER = armSlow ? 0.5 : 1.0;

            telemetry.addData("Driving Mode", isFieldOriented ? "Field-Oriented" : "Robot-Oriented");
            telemetry.addData("Driving Speed", driveSlow ? "50%" : "100%");
            telemetry.addData("Lift Position", MotorControl.intake.getCurrentPosition());
            telemetry.addData("Lift Debug Mode", liftDebug ? "On" : "Off");
            telemetry.addData("Lift Speed", liftSlow ? "50%" : "100%");
            telemetry.addData("Arm Position", MotorControl.lift.getCurrentPosition());
            telemetry.addData("Arm Speed", armSlow ? "50%" : "100%");

            telemetry.update();
        }
    }
}
