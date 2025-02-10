package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TankDrive
{
    ///// Create Motor Variables
    static DcMotor leftFront, leftBack, rightFront,  rightBack;
    /////

    ///// Name of Drive Motors on Driver Hub
    private static final String leftFrontName  = "leftFront",
                                leftBackName   = "leftBack",
                                rightFrontName = "rightFront",
                                rightBackName  = "rightBack";
    /////

    ///// Create Motion Variables
    double accel = 0.5; // Determine how fast the robot should go to full speed

    // Actual motor power
    double  leftPower  = 0,
            rightPower   = 0;

    // Intended motor power
    double  target_leftPower  = 0,
            target_rightPower   = 0;
    /////


    ///// Extra variables
    static Telemetry telemetry;
    /////

    // This method is used to initialize the motors/drivetrain of the robot
    public TankDrive(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Motor Objects
        TankDrive.leftFront  = hardwareMap.get(DcMotor.class, leftFrontName);
        TankDrive.leftBack   = hardwareMap.get(DcMotor.class, leftBackName);
        TankDrive.rightFront = hardwareMap.get(DcMotor.class, rightFrontName);
        TankDrive.rightBack  = hardwareMap.get(DcMotor.class, rightBackName);
        // Instantiate Telemetry
        TankDrive.telemetry = telemetry;

        // If the joysticks aren't touched, the robot won't move (set to BRAKE)
        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Instantiate Telemetry
        TankDrive.telemetry = telemetry;

        // Display Message on Screen
        telemetry.addData("initializing", "motors");

        // Reverse Motor Directions for Positive Values
        leftBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("reversing", "motors");
    }

    // This method is used to set the drive motor's powers
    public void ActiveDrive(double leftStickY, double rightStickY, double DRIVETRAIN_SPEED)
    {
        // Set intended power
        target_leftPower  = leftStickY;
        target_rightPower = rightStickY;

        // Apply acceleration to the motor powers
        leftPower  += accel * (target_leftPower  - leftPower);
        rightPower += accel * (target_rightPower - rightPower);

        // Set motor powers to desired values
        leftFront .setPower(leftPower  * DRIVETRAIN_SPEED);
        leftBack  .setPower(leftPower  * DRIVETRAIN_SPEED);
        rightFront.setPower(rightPower * DRIVETRAIN_SPEED);
        rightBack .setPower(rightPower * DRIVETRAIN_SPEED);

        telemetry.addData("Left", "%4.2f", leftPower);
        telemetry.addData("Right", "%4.2f", rightPower);
    }

    public void InitAuto()
    {
        //Stops and resets the encoder on the motors
        leftFront .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets mode to run without the encoder (to maximize motor efficiency/power)
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("drive motors' mode set","");
    }
}