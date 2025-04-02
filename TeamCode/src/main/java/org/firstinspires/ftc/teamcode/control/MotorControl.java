package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class MotorControl
{
    ///// Create Motor Variables
    public static DcMotorEx intakeMotor, lift;
    /////

    ///// Name of Control Motors on Driver Hub
    private static final String intakeName = "intake",
                                liftName = "lift";
    /////

    ///// Create and Define Motion Variables

    // Intake Variables
    public static double INTAKE_SPEED = 1.0;
    public enum IntakeDirection {in, out, none}

    // Lift Variables
    public int currentLiftPos;
    public int minLiftPos = 0;
    public int maxLiftPos = 1000;

    public double getLIFT_SPEED() {
        return LIFT_SPEED;
    }

    public void setLIFT_SPEED(double LIFT_SPEED) {
        this.LIFT_SPEED = LIFT_SPEED;
    }

    public enum LiftDirection {up, down}
    public enum LiftHeight {idle, intake, score}
    int idle_pos = 0,
        intake_pos = 100,
        score_pos = 500;
    private double LIFT_SPEED = 1.0;


    public boolean isLiftRunning = false;

    // Initialize the map
    Map<LiftHeight, Integer> liftPositions = new HashMap<>();

    /////
    private static final double[] liftPIDF = {0.006,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f
    /////

    ///// Create and Define Timer Variables to let the motors have time to run to position
    private static final ElapsedTime runtime = new ElapsedTime();
    /////

    ///// Extra variables
    static Telemetry telemetry;
    /////

    // This method is used to initialize the motors of the robot
    public MotorControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Motor Objects
        MotorControl.intakeMotor = hardwareMap.get(DcMotorEx.class, intakeName);
        MotorControl.lift = hardwareMap.get(DcMotorEx.class, liftName);
        // Instantiate Telemetry
        MotorControl.telemetry = telemetry;
        // Initialize the Map for liftPositions
        liftPositions.put(LiftHeight.idle, idle_pos);
        liftPositions.put(LiftHeight.intake, intake_pos);
        liftPositions.put(LiftHeight.score, score_pos);

        // If the joysticks aren't touched, the robot won't move (set to BRAKE)
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset Motor Encoders
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PIDControllers to the variables in the array
        ///// Create PIDF Variables
        PIDController liftPIDController = new PIDController(liftPIDF[0], liftPIDF[1], liftPIDF[2]);

        // Display Message on Screen
        telemetry.addData("Motor Status", "Initialized");
    }

    // This method is used to activate the intake
    public static void intake(IntakeDirection direction, double SPEED_MULTIPLIER)
    {
        if(direction == IntakeDirection.in)
        {
            intakeMotor.setPower(INTAKE_SPEED * SPEED_MULTIPLIER);
        }
        else if (direction == IntakeDirection.out)
        {
            intakeMotor.setPower(-INTAKE_SPEED * SPEED_MULTIPLIER);
        }
        else
        {
            intakeMotor.setPower(0);
        }

        telemetry.addData("Intake Status", direction);
    }

    // This method is used to reject a wrong colored game element
    public static void reject()
    {
        intake(IntakeDirection.none, 1);
        while(runtime.milliseconds() < 500)
        {
            telemetry.addData("Color Sort", "REJECTING");
        }
        intake(IntakeDirection.in, 1);
    }
}
