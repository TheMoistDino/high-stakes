package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.HolonomicDrive;
import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;

@Disabled
@TeleOp(name = "Sample TeleOp", group = "TeleOp")
public class SampleTeleOp extends LinearOpMode
{
    // Variables for Method Calling
    HolonomicDrive holonomicDrive;
    //TankDrive tankDrive;
    ServoControl servoControl;
    MotorControl motorControl;
    //////////////////////

    double DRIVETRAIN_SPEED = 1.0;
    double LIFT_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // For Holonomic Drive
        holonomicDrive = new HolonomicDrive(hardwareMap, telemetry);
        //////////////////////

        // For Tank Drive
        //tankDrive = new TankDrive(hardwareMap, telemetry);
        //////////////////////

        // For Servo Control (Claws)
        servoControl = new ServoControl(hardwareMap, telemetry);
        //////////////////////

        // For Motor Control (Lift)
        motorControl = new MotorControl(hardwareMap, telemetry);
        //////////////////////


        telemetry.addData("robot ready","");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        while(opModeIsActive())
        {
            // For Holonomic Drive
            holonomicDrive.ActiveDriveRO(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED);
            //////////////////////

            // For Tank Drive
            //tankDrive.ActiveDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
            //////////////////////

            // Button to slow down driving
            DRIVETRAIN_SPEED = gamepad1.left_bumper ? 0.5 : 1.0;

            // Button to slow down lift
            LIFT_SPEED = gamepad2.left_bumper ? 0.5 : 1.0;

            // Button to control the claw servo


            telemetry.update();
        }
    }
}
