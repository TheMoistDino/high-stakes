package org.firstinspires.ftc.teamcode.control;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class HolonomicDrive
{
    ///// Create Motor Variables
    static DcMotor leftFront, leftBack, rightFront, rightBack;
    /////

    ///// Name of Drive Motors on Driver Hub
    private static final String leftFrontName  = "leftFront",
                                leftBackName   = "leftBack",
                                rightFrontName = "rightFront",
                                rightBackName  = "rightBack";
    /////

    ///// Create IMU/gyro variables
    // static BNO055IMU imu;
    static IMU imu;
    // Orientation angles;
    YawPitchRollAngles robotOrientation;
    double initYaw;
    double adjustedYaw;

    ///// Create Motion Variables
    double accel = 0.5; // Determine how fast the robot should go to full speed

    // Actual motor power
    double  leftFront_power  = 0,
            leftBack_power   = 0,
            rightFront_power = 0,
            rightBack_power  = 0;

    // Intended motor power
    double  target_leftFront_power  = 0,
            target_leftBack_power   = 0,
            target_rightFront_power = 0,
            target_rightBack_power  = 0;
    /////

    ///// Create Input Variables
    double x, y, turn;
    /////

    ///// Physical to Digital Variables
    //Get the motor's ppr and multiply it by 4 to get counts per motor rev
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    //If you have no drive gear reduction leave at 1
    static final double DRIVE_GEAR_REDUCTION = 1;
    //Input the diameter measure in inches
    static final double WHEEL_DIAMETER_INCHES = (10.4) / (2.54);
    //How many times the motor goes per a inch
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private PIDController pidController;
    private static final double[] drivePIDF = {0,0,0,0}; // index 0 = p, 1 = i, 2 = d, 3 = f

    ///// Create and Define Timer Variables to let the motors have time to run to position
    private final ElapsedTime runtime = new ElapsedTime();
    /////


    ///// Extra variables
    static Telemetry telemetry;
    /////


    // This method is used to initialize the motors/drivetrain of the robot to holonomic/mecanum drive
    public HolonomicDrive(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Motor Objects
        HolonomicDrive.leftFront  = hardwareMap.get(DcMotor.class, leftFrontName);
        HolonomicDrive.leftBack   = hardwareMap.get(DcMotor.class, leftBackName);
        HolonomicDrive.rightFront = hardwareMap.get(DcMotor.class, rightFrontName);
        HolonomicDrive.rightBack  = hardwareMap.get(DcMotor.class, rightBackName);

        // Instantiate IMU/gyro Objects
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;

        HolonomicDrive.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //initYaw = angles.firstAngle;

        robotOrientation = imu.getRobotYawPitchRollAngles();
        initYaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        // Instantiate Telemetry
        HolonomicDrive.telemetry = telemetry;

        // If the joysticks aren't touched, the robot won't move (set to BRAKE)
        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Display Message on Screen
        telemetry.addData("Motors", "Initialized");

        // Reverse Motor Directions for Positive Values
        leftBack .setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Motors", "Reversed");
    }

    // This method is used to initialize the drive motors' modes
    public void InitAuto()
    {
        //Stops and resets the encoder on the motors
        leftFront .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets mode to run without the encoder (to maximize motor efficiency/power)
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("drive motors' mode set","");
    }

    // This method is used for robot-oriented driving in TeleOp
    public void ActiveDriveRO(double leftStickX, double leftStickY, double rightStickX, double SPEED_MULTIPLIER)
    {
        // Cool vector math to calculate power to the drive motors
        x    = leftStickX * 1.05;
        y    = -leftStickY;
        turn = rightStickX;

        // Input variables in new version are power and theta
        double theta = Math.atan2(y,x);
        double power = Math.hypot(x,y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Combine variables to find power and set the intended power
        target_leftFront_power  = (power * cos / max + turn);
        target_leftBack_power   = (power * sin / max + turn);
        target_rightFront_power = (power * sin / max - turn);
        target_rightBack_power  = (power * cos / max - turn);

        if ((power + Math.abs(turn)) > 1.0) // Limit motor's powers to 100%
        {
            target_leftFront_power  /= power + Math.abs(turn);
            target_rightFront_power /= power + Math.abs(turn);
            target_leftBack_power   /= power + Math.abs(turn);
            target_rightBack_power  /= power + Math.abs(turn);
        }

        // Apply acceleration to the motor powers
        leftFront_power  += accel * (target_leftFront_power  - leftFront_power);
        leftBack_power   += accel * (target_leftBack_power   - leftBack_power);
        rightFront_power += accel * (target_rightFront_power - rightFront_power);
        rightBack_power  += accel * (target_rightBack_power  - rightBack_power);

        // Set motor powers to desired values
        leftFront .setPower(leftFront_power  * SPEED_MULTIPLIER);
        leftBack  .setPower(leftBack_power   * SPEED_MULTIPLIER);
        rightFront.setPower(rightFront_power * SPEED_MULTIPLIER);
        rightBack .setPower(rightBack_power  * SPEED_MULTIPLIER);

        // Display motor power
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFront_power, rightFront_power);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBack_power, rightBack_power);
    }

    // This method is used for field-oriented driving in TeleOp
    public void ActiveDriveFO(double leftStickX, double leftStickY, double rightStickX, double SPEED_MULTIPLIER)
    {
//        // adjustedYaw = angles.firstAngle - initYaw;
//        adjustedYaw = robotOrientation.getYaw(AngleUnit.RADIANS) - initYaw;
//
//        // double zeroedYaw = -initYaw + angles.firstAngle;
//        double zeroedYaw = -initYaw + robotOrientation.getYaw(AngleUnit.RADIANS);
//
//        // Cool vector math to calculate power to the drive motors
//        x    = leftStickX;
//        y    = -leftStickY;
//        turn = rightStickX;
//
//        double theta = Math.atan2(y,x); // aka angle
//
//        double realTheta = ((2*Math.PI) - zeroedYaw) + theta;
//
//        double power = Math.hypot(x,y);
//
//        double sin = Math.sin((realTheta) - (Math.PI / 4));
//        double cos = Math.cos((realTheta) - (Math.PI / 4));
//        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));
//
//        // Combine variables to find power and set the intended power
//        target_leftFront_power  = (power * cos / maxSinCos + turn);
//        target_leftBack_power   = (power * sin / maxSinCos + turn);
//        target_rightFront_power = (power * sin / maxSinCos - turn);
//        target_rightBack_power  = (power * cos / maxSinCos - turn);
//
//        if ((power + Math.abs(turn)) > 1.0) // Limit motor's powers to 100%
//        {
//            target_leftFront_power  /= power + Math.abs(turn);
//            target_rightFront_power /= power + Math.abs(turn);
//            target_leftBack_power   /= power + Math.abs(turn);
//            target_rightBack_power  /= power + Math.abs(turn);
//        }

        // Cool vector math to calculate power to the drive motors
        x    = leftStickX;
        y    = -leftStickY;
        turn = rightStickX;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        rotX = rotX * 1.05; // Counteract Strafing

        // Max is the largest motor power or 1, limiting all motor's powers to 100%
        double max = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);
        target_leftFront_power  = (rotY + rotX + turn) / max;
        target_leftBack_power   = (rotY - rotX + turn) / max;
        target_rightFront_power = (rotY - rotX - turn) / max;
        target_rightBack_power  = (rotY + rotX - turn) / max;

        // Apply acceleration to the motor powers
        leftFront_power  += accel * (target_leftFront_power  - leftFront_power);
        leftBack_power   += accel * (target_leftBack_power   - leftBack_power);
        rightFront_power += accel * (target_rightFront_power - rightFront_power);
        rightBack_power  += accel * (target_rightBack_power  - rightBack_power);

        // Set motor powers to desired values
        leftFront .setPower(leftFront_power  * SPEED_MULTIPLIER);
        leftBack  .setPower(leftBack_power   * SPEED_MULTIPLIER);
        rightFront.setPower(rightFront_power * SPEED_MULTIPLIER);
        rightBack .setPower(rightBack_power  * SPEED_MULTIPLIER);

        // Display motor power
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFront_power, rightFront_power);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBack_power, rightBack_power);
    }

    public void ActiveDrive(double leftStickX, double leftStickY, double rightStickX, double SPEED_MULTIPLIER, boolean isFieldOriented)
    {
        if(isFieldOriented)
        {
            ActiveDriveFO(leftStickX, leftStickY, rightStickX, SPEED_MULTIPLIER);
        }
        else
        {
            ActiveDriveRO(leftStickX, leftStickY, rightStickX, SPEED_MULTIPLIER);
        }
    }

    // This method is used to drive the robot forward some number of inches forward in Auto
    public void ForwardDrive(double inchesForward, double maxPower, double timeoutS, int h)
    {
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tells the motors the target position
        leftFront .setTargetPosition((leftFront .getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH)));
        leftBack  .setTargetPosition((leftBack  .getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH)));
        rightFront.setTargetPosition((rightFront.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH)));
        rightBack .setTargetPosition((rightBack .getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH)));

        //Sets the mode of the motors to run to the positions
        leftFront .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack  .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack .setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Resets timer
        runtime.reset();

        //Sets power to the motors
        leftFront .setPower(maxPower);
        leftBack  .setPower(maxPower);
        rightFront.setPower(maxPower);
        rightBack .setPower(maxPower);

        //Puts the current position of the motors and puts it onto the telemetry
        while ((runtime.seconds() < timeoutS) &&  (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("currently running","");
            telemetry.update();
        }
        //After it runs to the position it stops
        leftFront .setPower(0);
        leftBack  .setPower(0);
        rightFront.setPower(0);
        rightBack .setPower(0);

        //Sets mode back to run without the encoder
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void ForwardDrivePID(double inchesForward, double timeoutS)
    {
        // Resets timer
        runtime.reset();

        // Gets positions of drive motors
        int lfPos = leftFront .getCurrentPosition(), lbPos = leftBack .getCurrentPosition(),
            rfPos = rightFront.getCurrentPosition(), rbPos = rightBack.getCurrentPosition();

        // Sets the target positions for each drive motor
        int lfTarget = lfPos + (int) (-inchesForward * COUNTS_PER_INCH),
            lbTarget = lbPos + (int) (-inchesForward * COUNTS_PER_INCH),
            rfTarget = rfPos + (int) (-inchesForward * COUNTS_PER_INCH),
            rbTarget = rbPos + (int) (-inchesForward * COUNTS_PER_INCH);

        while ((runtime.seconds() < timeoutS) &&  (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            // Sets controller PID to the variables set
            pidController.setPID(drivePIDF[0],drivePIDF[1],drivePIDF[2]);

            // Gets positions of drive motors
            lfPos = leftFront .getCurrentPosition();
            lbPos = leftBack  .getCurrentPosition();
            rfPos = rightFront.getCurrentPosition();
            rbPos = rightBack .getCurrentPosition();

            // Calculates how much power to input based on the position to run to the desired target
            double lfPID = pidController.calculate(lfPos, lfTarget),
                   lbPID = pidController.calculate(lbPos, lbTarget),
                   rfPID = pidController.calculate(rfPos, rfTarget),
                   rbPID = pidController.calculate(rbPos, rbTarget);

            double lfFF = lfTarget * drivePIDF[3],
                   lbFF = lbTarget * drivePIDF[3],
                   rfFF = rfTarget * drivePIDF[3],
                   rbFF = rbTarget * drivePIDF[3];

            double lfPower = lfPID + lfFF,
                   lbPower = lbPID + lbFF,
                   rfPower = rfPID + rfFF,
                   rbPower = rbPID + rbFF;

            //Sets power to the motors
            leftFront .setPower(lfPower);
            leftBack  .setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack .setPower(rbPower);

            telemetry.addData("currently running","");
            telemetry.update();
        }

        //After it runs to the position it stops
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        telemetry.addData("finished driving","");
        telemetry.update();
    }

    public void ForwardDrive(double inches, double power, double timeoutS)
    {
        int target = (int)(inches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightBack.setTargetPosition(target);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        while((runtime.seconds() < timeoutS) && (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()))
        {
            telemetry.addData("currently running","");
            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void TimerStraight(double power, double timeoutS)
    {
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        while(runtime.seconds() < timeoutS)
        {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);

            telemetry.addData("currently running","");
            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void TimerStrafe(double power, double timeoutS)
    {
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        while(runtime.seconds() < timeoutS)
        {
            leftFront.setPower(-power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(-power);

            telemetry.addData("currently running","");
            telemetry.update();
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
