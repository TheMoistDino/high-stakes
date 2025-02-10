package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoControl
{
    ///// Create Servo Variables
    static ServoImplEx slideLeft, slideRight;
    static ServoImplEx pitch, roll, clawIntake, clawOuttake;
    /////

    ///// Name of Servos on Driver Hub
    private static final String slideLeftName = "slideLeft",
                                slideRightName = "slideRight";
    private static final String pitchName = "pitch",
                                rollName = "roll",
                                clawIntakeName = "clawIntake",
                                clawOuttakeName = "clawOuttake";
    /////

    ///// Create and Define Motion Variables
    static boolean isGrabIntake, isGrabOuttake, isPitchOut;
    public enum SlidesPosition {zero, half, max}
    public enum SlidesDirection {in, out}
    public enum RollDirection {left, right}
    static final double clawIntakeClosePos = 0.4, // Change to closed claw position
                        clawIntakeOpenPos  = 0.15, // Change to open claw position
                        clawOuttakeClosePos = 0.3, // Change to closed claw position
                        clawOuttakeOpenPos = 0.05, // Change to open claw position
                        slideLeftZeroPos = 0.95,
                        slideLeftHalfPos = 0.8,
                        slideLeftMaxPos = 0.6,
                        slideRightZeroPos = 0.0,
                        slideRightHalfPos = 0.15,
                        slideRightMaxPos = 0.35,
                        pitchOutPos = 0.18,
                        pitchInPos = 0.95,
                        rollDefaultPos = 0.65;
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
        ServoControl.clawIntake = hardwareMap.get(ServoImplEx.class, clawIntakeName);
        ServoControl.clawOuttake = hardwareMap.get(ServoImplEx.class, clawOuttakeName);
        ServoControl.slideLeft = hardwareMap.get(ServoImplEx.class, slideLeftName);
        ServoControl.slideRight = hardwareMap.get(ServoImplEx.class, slideRightName);

        ServoControl.pitch = hardwareMap.get(ServoImplEx.class, pitchName);
        ServoControl.roll = hardwareMap.get(ServoImplEx.class, rollName);

        // Increase max range of the servos
        clawIntake.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawOuttake.setPwmRange(new PwmControl.PwmRange(500, 2500));
        slideLeft.setPwmRange(new PwmControl.PwmRange(500, 2500));
        slideRight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        pitch.setPwmRange(new PwmControl.PwmRange(500, 2500));
        roll.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Instantiate Telemetry
        ServoControl.telemetry = telemetry;

        // Disable servo power by default
        StopServos();

        // Display Message on Screen
        telemetry.addData("servos", "initializing");
    }

    // This method is used to "turn on" the servos (help prevent movement between AUTO and TELEOP periods)
    public void StartServos()
    {
        // Enable servo power
        clawIntake.setPwmEnable();
        clawOuttake.setPwmEnable();
        slideLeft.setPwmEnable();
        slideRight.setPwmEnable();
        pitch.setPwmEnable();
        roll.setPwmEnable();

        // Start the slides in initial positions
        slideLeft.setPosition(slideLeftZeroPos);
        slideRight.setPosition(slideRightZeroPos);
        pitch.setPosition(pitchInPos);
        roll.setPosition(rollDefaultPos);

        // Start the claw and bucket in initial positions
        isGrabIntake = false;
        isGrabOuttake = false;
        clawIntake.setPosition(clawIntakeOpenPos);
        clawOuttake.setPosition(clawIntakeOpenPos);

        // Display Message on Screen
        telemetry.addData("servos", "started");
    }

    // This method is used to "turn off" the servos (help prevent movement between AUTO and TELEOP periods)
    public void StopServos()
    {
        // Disable servo power
        clawIntake.setPwmDisable();
        clawOuttake.setPwmDisable();
        slideLeft.setPwmDisable();
        slideRight.setPwmDisable();
        pitch.setPwmDisable();
        roll.setPwmDisable();
        // slide.setPwmDisable();

        // Display Message on Screen
        telemetry.addData("stopping", "servos");
    }

    // This method is used to open/close the claw servo
    public void GrabIntake()
    {
        // Restart timer
        runtime.reset();

        // Open/Close the Intake Claw
        isGrabIntake = !isGrabIntake;
        clawIntake.setPosition(isGrabIntake ? clawIntakeOpenPos : clawIntakeClosePos);

        // Give time for the servo to run to position
        while(runtime.milliseconds() < timeout)
        {
            telemetry.addData("intake claw running for:", runtime.milliseconds());
            telemetry.update();
        }
        telemetry.addData("isGrabIntake", isGrabIntake);
        telemetry.update();
    }

    // This method is used to dump the bucket servo
    public void GrabOuttake()
    {
        // Restart timer
        runtime.reset();

        // Open/Close the Claw
        isGrabOuttake = !isGrabOuttake;
        clawOuttake.setPosition(isGrabOuttake ? clawOuttakeOpenPos : clawOuttakeClosePos);

        // Give time for the servo to run to position
        while(runtime.milliseconds() < timeout)
        {
            telemetry.addData("bucket running for:", runtime.milliseconds());
            telemetry.update();
        }
        telemetry.addData("isGrabOuttake", isGrabOuttake);
        telemetry.update();
    }

    public void PitchControl() {
        // Restart timer
        runtime.reset();

        // Open/Close the Claw
        isPitchOut = !isPitchOut;
        pitch.setPosition(isPitchOut ? pitchOutPos : pitchInPos);

        // Give time for the servo to run to position
        while (runtime.milliseconds() < timeout) {
            telemetry.addData("bucket running for:", runtime.milliseconds());
            telemetry.update();
        }
        telemetry.addData("isPitchOut", isPitchOut);
        telemetry.update();
    }

    public void SlidesPreset(SlidesPosition position) {
        // Restart timer
        runtime.reset();

        if (position == SlidesPosition.zero)
        {
            slideLeft.setPosition(slideLeftZeroPos);
            slideRight.setPosition(slideRightZeroPos);
        }
        else if (position == SlidesPosition.half)
        {
            slideLeft.setPosition(slideLeftHalfPos);
            slideRight.setPosition(slideRightHalfPos);
        }
        else if (position == SlidesPosition.max)
        {
            slideLeft.setPosition(slideLeftMaxPos);
            slideRight.setPosition(slideRightMaxPos);
        }

        // Give time for the servo to run to position
        while (runtime.milliseconds() < timeout) {
            telemetry.addData("slides", runtime.milliseconds());
            telemetry.update();
        }
        telemetry.addData("slides:", position);
        telemetry.update();
    }

    public void SlidesControl(SlidesDirection direction)
    {
        if (direction == SlidesDirection.in)
        {
            slideLeft.setPosition(slideLeft.getPosition() + 0.03);
            slideRight.setPosition(slideRight.getPosition() - 0.03);
        }
        else
        {
            slideLeft.setPosition(slideLeft.getPosition() - 0.03);
            slideRight.setPosition(slideRight.getPosition() + 0.03);
        }
    }

    public void RollControl(RollDirection direction)
    {
        if (direction == RollDirection.left)
        {
            roll.setPosition(roll.getPosition() + 0.05);
        }
        else
        {
            roll.setPosition(roll.getPosition() - 0.05);
        }
    }
}
