package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class MotorControl
{
    ///// Create Motor Variables
    public static DcMotor liftLeft, liftRight, outtakeArm;
    /////

    ///// Name of Control Motors on Driver Hub
    private static final String liftLeftName = "liftLeft",
                                liftRightName = "liftRight",
                                outtakeArmName = "outtakeArm";
    /////

    ///// Create and Define Motion Variables
    // Lift Variables
    double liftAccel = 0.5;
    double liftPower = 0.0;
    double max_liftPower = 1.0;
    public int currentLiftPos;
    public int targetLiftPos = 0;
    public int minLiftPos = 0;
    public int maxLiftPos = 4100;
    public enum LiftDirection {up, down}
    public enum LiftHeight {high_basket, low_basket, high_chamber, low_chamber, zero}
    int high_basket_pos = 4100, low_basket_pos = 1950,
        high_chamber_pos = 3000, low_chamber_pos = 1000,
        zero_pos = 0;

    public boolean isLiftRunning = false;

    // Initialize the map
    Map<LiftHeight, Integer> liftPositions = new HashMap<>();

    // Arm Variables
    double armAccel = 0.25;
    double armPower = 0.0;
    double max_armPower = 0.3;
    public int currentArmPos;
    public enum ArmDirection {forward, backward, setup}

    public boolean isArmRunning = false;
    /////

    ///// Create PIDF Variables
    private final PIDController armPIDController;
    private final PIDController liftPIDController;
    private static final double[] armPIDF = {0,0.01,0,0.004}; // index 0 = p, 1 = i, 2 = d, 3 = f
    private static final double[] liftPIDF = {0.006,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f
    /////

    ///// Create and Define Timer Variables to let the motors have time to run to position
    private final ElapsedTime runtime = new ElapsedTime();
    /////

    ///// Extra variables
    static Telemetry telemetry;
    /////

    // This method is used to initialize the motors of the robot
    public MotorControl(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Motor Objects
        MotorControl.liftLeft = hardwareMap.get(DcMotor.class, liftLeftName);
        MotorControl.liftRight = hardwareMap.get(DcMotor.class, liftRightName);
        MotorControl.outtakeArm = hardwareMap.get(DcMotor.class, outtakeArmName);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeArm.setDirection(DcMotor.Direction.REVERSE);
        // Instantiate Telemetry
        MotorControl.telemetry = telemetry;
        // Initialize the Map for liftPositions
        liftPositions.put(LiftHeight.high_basket, high_basket_pos);
        liftPositions.put(LiftHeight.low_basket, low_basket_pos);
        liftPositions.put(LiftHeight.high_chamber, high_chamber_pos);
        liftPositions.put(LiftHeight.low_chamber, low_chamber_pos);
        liftPositions.put(LiftHeight.zero, zero_pos);

        // If the joysticks aren't touched, the robot won't move (set to BRAKE)
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Motor Encoders
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set PIDControllers to the variables in the array
        liftPIDController = new PIDController(liftPIDF[0],liftPIDF[1],liftPIDF[2]);
        armPIDController = new PIDController(armPIDF[0],armPIDF[1],armPIDF[2]);

        // Display Message on Screen
        telemetry.addData("motors", "initializing");
    }

    // This method is used to lock the position of the lift
    public void LockLift()
    {
        currentLiftPos = liftLeft.getCurrentPosition();
        liftLeft.setTargetPosition(targetLiftPos);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setTargetPosition(targetLiftPos);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(max_liftPower);
        liftRight.setPower(max_liftPower);
    }

    public void StopAndReturnLift()
    {
        // Calculate the distance to each limit
        int distanceToMin = Math.abs(currentLiftPos - minLiftPos);
        int distanceToMax = Math.abs(currentLiftPos - maxLiftPos);

        // Determine the closest limit
        int targetPosition = (distanceToMin < distanceToMax) ? (minLiftPos + 20) : (maxLiftPos - 20);
        targetLiftPos = targetPosition;

        // Move the lift to the closest limit
        liftLeft.setTargetPosition(targetPosition);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(max_liftPower); // Assuming you have a max_liftPower variable
    }

    // This method is used to lock the position of the arm
    public void LockArm()
    {
        armPower = 0;
        currentArmPos = outtakeArm.getCurrentPosition();
        outtakeArm.setTargetPosition(currentArmPos);
        outtakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeArm.setPower(max_armPower);
    }

    // This method is used to move the lift
    public void MoveLift(LiftDirection liftDirection, double LIFT_SPEED)
    {
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (liftDirection)
        {
            case up:
                liftPower = 1;
                targetLiftPos += 10;
                break;
            case down:
                liftPower = -1;
                targetLiftPos -= 10;
                break;
        }

        //liftLeft.setTargetPosition(targetLiftPos);
        //liftRight.setTargetPosition(targetLiftPos);
        //liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(liftPower * LIFT_SPEED * 0.87);
        liftRight.setPower(liftPower * LIFT_SPEED);

        targetLiftPos = liftLeft.getCurrentPosition();
    }

    // This method is used to move the arm
    public void MoveArm(ArmDirection armDirection, double LIFT_SPEED)
    {
        outtakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (armDirection)
        {
            case forward:
                max_armPower = 0.2;
                armPower += armAccel * (max_armPower - armPower);
                break;
            case backward:
                max_armPower = -0.2;
                armPower += armAccel * (max_armPower - armPower);
                break;
        }

        outtakeArm.setPower(max_armPower * LIFT_SPEED);

        currentArmPos = outtakeArm.getCurrentPosition();
    }

    public void ArmControl(ArmDirection direction, double armSpeed)
    {
        outtakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        switch (direction)
        {
            case forward:
                armPower = max_armPower;
                break;
            case backward:
                armPower = -max_armPower;
                break;
            case setup:
                armPower = 0;
        }

        outtakeArm.setPower(armPower * armSpeed);
    }


    // This method is used to make the lift go to a specified position
    public void LiftToPosition(LiftHeight liftHeight, double timeoutSeconds)
    {
        // Get the target position
        int target = liftPositions.getOrDefault(liftHeight, 0);

        liftPIDController.setPID(liftPIDF[0],liftPIDF[1],liftPIDF[2]);

        // Initialize the lift motor to the correct mode (to maximize power)
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Resets timer
        runtime.reset();

        // Gets the position of the lift motor
        currentLiftPos = liftLeft.getCurrentPosition();

        while ((runtime.seconds() > 0) && (runtime.seconds() < timeoutSeconds))
        {
            // Gets position of lift motor
            currentLiftPos = liftLeft.getCurrentPosition();

            // Calculates the power of the lift motor
            double liftPID = liftPIDController.calculate(currentLiftPos, target);
            double ff = currentLiftPos * liftPIDF[3];

            // Sets the power of the lift motor
            double power = liftPID + ff;
            liftLeft.setPower(power);

            isLiftRunning = true;

            // Update current state to telemetry
            telemetry.addData("currently running for", runtime.seconds());
            telemetry.addData("current position", liftLeft.getCurrentPosition());
            telemetry.addData("target",target);
        }

        // After the lift runs to position, it stops
        liftLeft.setPower(0);

        isLiftRunning = false;

        telemetry.addData("final position", liftLeft.getCurrentPosition());
        telemetry.addData("target",target);
        telemetry.update();
    }

    // This method is used to make the arm go to a specified position
    public void ArmToPosition(int target, double timeoutSeconds)
    {
        // Initialize the arm motor to the correct mode (to maximize power)
        outtakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Resets timer
        runtime.reset();

        // Gets the position of the lift motor
        currentArmPos = outtakeArm.getCurrentPosition();

        while ((runtime.seconds() < timeoutSeconds) && (outtakeArm.isBusy()))
        {
            // Gets position of lift motor
            currentArmPos = outtakeArm.getCurrentPosition();

            // Calculates the power of the lift motor
            double armPID = armPIDController.calculate(currentArmPos, target);
            double ff = currentArmPos * armPIDF[3];

            // Sets the power of the lift motor
            double power = armPID + ff;
            outtakeArm.setPower(power);

            isArmRunning = true;

            // Update current state to telemetry
            telemetry.addData("currently running","");
            telemetry.update();
        }

        // After the lift runs to position, it stops
        outtakeArm.setPower(0);

        isArmRunning = false;

        telemetry.addData("final position", outtakeArm.getCurrentPosition());
        telemetry.update();
    }
}
