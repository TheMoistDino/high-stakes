package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.SensorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Red Left", group = "Auto", preselectTeleOp = "TeleOp")
public class RedLeft extends LinearOpMode
{
    // Variables used for Method Calling
    MotorControl motor;
    ServoControl servo;
    SensorControl sensor;
    /////////////////////////

    public class Clamp
    {
        public class Toggle implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                servo.ToggleClamp();
                return false;
            }
        }
        public Action toggle() { return new Toggle(); }
    }

    public class Intake
    {
        public class In implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                motor.intake(MotorControl.IntakeDirection.in, 1);
                return false;
            }
        }
        public Action in() { return new In(); }

        public class Out implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                motor.intake(MotorControl.IntakeDirection.out, 1);
                return false;
            }
        }
        public Action out() { return new Out(); }

        public class Stop implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                motor.intake(MotorControl.IntakeDirection.none, 1);
                return false;
            }
        }
        public Action stop() { return new Stop(); }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize control objects
        motor = new MotorControl(hardwareMap, telemetry);
        servo = new ServoControl(hardwareMap, telemetry);
        sensor = new SensorControl(hardwareMap, telemetry);
        /////////////////////////

        // Initialize RoadRunner
        Clamp clamp = new Clamp();
        Intake intake = new Intake();

        Pose2d firstPose = new Pose2d(-52, 24, Math.toRadians(180));
        Pose2d secondPose = new Pose2d(-32, 24, Math.toRadians(180));
        Pose2d thirdPose = new Pose2d(-48, 24, Math.toRadians(45));

        MecanumDrive drive = new MecanumDrive(hardwareMap, firstPose);

        TrajectoryActionBuilder step1 = drive.actionBuilder(firstPose)
                .lineToX(-32)
                .waitSeconds(0.3);
        TrajectoryActionBuilder step2 = drive.actionBuilder(secondPose)
                .lineToXLinearHeading(-48, Math.toRadians(45));
        TrajectoryActionBuilder step3 = drive.actionBuilder(thirdPose)
                .lineToY(36)
                .lineToY(50, new TranslationalVelConstraint(10));


        /////////////////////////

        servo.StartServos();

        telemetry.addData("Robot Status", "Ready");
        telemetry.update();

        // Wait for the play button to be pressed
        waitForStart();

        // Run Road Runner Actions
        Actions.runBlocking(
                new SequentialAction
                (
                        step1.build(),
                        clamp.toggle(),
                        new SleepAction(0.2),
                        step2.build(),
                        new SleepAction(0.2),
                        intake.in(),
                        step3.build(),
                        new SleepAction(3.0),
                        intake.stop()
                )
        );


        // Stop all servos
        servo.StopServos();

        telemetry.addData("Robot Status", "Autonomous Complete");
        telemetry.update();

    }
}
