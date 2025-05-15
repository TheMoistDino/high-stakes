package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.MotorControl;
import org.firstinspires.ftc.teamcode.control.SensorControl;
import org.firstinspires.ftc.teamcode.control.ServoControl;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Red Right", group = "Auto", preselectTeleOp = "TeleOp")
public class RedRight2 extends LinearOpMode
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

    public class Doinker
    {
        public class Toggle implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                servo.ToggleDoinker();
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
        Doinker doinker = new Doinker();
        Intake intake = new Intake();

        Pose2d firstPose = new Pose2d(-62, -24, Math.toRadians(0));
        Pose2d secondPose = new Pose2d(-61,0,Math.toRadians(0));
        Pose2d thirdPose = new Pose2d(-28,-21,Math.toRadians(-210));
        Pose2d fourthPose = new Pose2d(-48, -24, Math.toRadians(-45));

        MecanumDrive drive = new MecanumDrive(hardwareMap, firstPose);

        TrajectoryActionBuilder step1 = drive.actionBuilder(firstPose)
                .lineToX(-60)
                .strafeToConstantHeading(new Vector2d(-60, 0))
                .waitSeconds(0.1)
                .lineToX(-61);
        TrajectoryActionBuilder step2 = drive.actionBuilder(secondPose)
                .lineToX(-55)
                .turnTo(Math.toRadians(-210))
                .strafeToLinearHeading(new Vector2d(-28,-21),Math.toRadians(-210))
                .waitSeconds(0.5);
        TrajectoryActionBuilder step3 = drive.actionBuilder(thirdPose)
                .strafeToLinearHeading(new Vector2d(-48,-24), Math.toRadians(-45));
        TrajectoryActionBuilder step4 = drive.actionBuilder(fourthPose)
                .lineToY(-36)
                .lineToY(-53, new TranslationalVelConstraint(10))
                .waitSeconds(0.5)
                .lineToY(-48, new TranslationalVelConstraint(10));


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
                        new ParallelAction(
                                new SleepAction(0.5),
                                intake.in()
                        ),
                        intake.stop(),
                        step2.build(),
                        clamp.toggle(),
                        step3.build(),
                        intake.in(),
                        step4.build(),
                        new SleepAction(1.0),
                        new ParallelAction(
                                intake.stop(),
                                clamp.toggle()
                        )
                )
        );


        // Stop all servos
        servo.StopServos();

        telemetry.addData("Robot Status", "Autonomous Complete");
        telemetry.update();

    }
}
