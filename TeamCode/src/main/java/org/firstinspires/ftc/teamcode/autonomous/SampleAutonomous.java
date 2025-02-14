package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Sample Autonomous", group = "Autonomous")
public class SampleAutonomous extends LinearOpMode {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    //Get the motor's ppr and multiply it by 4 to get counts per motor rev
    static final double COUNTS_PER_MOTOR_REV = 0;
    //If you have no drive gear reduction leave at 1
    static final double DRIVE_GEAR_REDUCTION = 1;
    //Input the diameter measure in inches
    static final double WHEEL_DIAMETER_INCHES = 0;
    //How many times the motor goes per a inch
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //Timer
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Hardware mapping the motor's
        leftFront = hardwareMap.dcMotor.get("frontLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightBack = hardwareMap.dcMotor.get("backRight");

        //Line 30-44 sets the motor behavior and modes
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        encoderDrive(1, 10, 10, 5);
        encoderDriveStrafe(1, 5, 5, 5);
    }
    //Procedure that drives with the encoders using different values
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        //Variables that will be used later to store the targets for each motor
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;


        if (opModeIsActive()) {
            //Stops and resets the encoder on the motors
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Calculates the targets of the motors
            newFrontLeftTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            //Tells the motors the target position
            leftFront.setTargetPosition(newFrontLeftTarget);
            leftBack.setTargetPosition(newBackLeftTarget);
            rightFront.setTargetPosition(newFrontRightTarget);
            rightBack.setTargetPosition(newBackRightTarget);

            //Sets the mode of the motors to run to the positions
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Resets timer
            runtime.reset();
            //Sets power to the motors
            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            //Puts the current position of the motors and puts it onto the telemetry
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        leftFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        rightBack.getCurrentPosition());
                telemetry.update();
            }
            //After it runs to the position it stops
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            //Sets mode back to run using the encoder
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);
        }
    }

    public void encoderDriveStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        //Variables that will be used later to store the targets for each motor
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            //Stops and resets the encoder on the motors
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Calculates the targets of the motors
            newFrontLeftTarget = leftFront.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = rightBack.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);

            //Tells the motors the target position
            leftFront.setTargetPosition(newFrontLeftTarget);
            leftBack.setTargetPosition(newBackLeftTarget);
            rightFront.setTargetPosition(newFrontRightTarget);
            rightBack.setTargetPosition(newBackRightTarget);

            //Sets the mode of the motors to run to the positions
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Resets timer
            runtime.reset();

            //Sets power to the motors
            leftFront.setPower(Math.abs(-speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(-speed));

            //Puts the current position of the motors and puts it onto the telemetry
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&  (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        leftFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        rightBack.getCurrentPosition());
                telemetry.update();
            }
            //After it runs to the position it stops
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            //Sets mode back to run using the encoder
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);
        }
    }
}