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

@Autonomous(name = "Red Left", group = "Auto", preselectTeleOp = "TeleOp")
public class RedLeft2 extends LinearOpMode
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
    public class Sensor
    {
        public class CheckDistance implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket packet)
            {
                if(!sensor.checkDistance())
                {
                    Pose2d intPose1 = new Pose2d(-24-(6.5*Math.cos(Math.toRadians(30))),24-(6.5*Math.sin(Math.toRadians(30))), Math.toRadians(-150));
                    Pose2d intPose2 = new Pose2d(-24-(8*Math.cos(Math.toRadians(30))),24-(8*Math.sin(Math.toRadians(30))), Math.toRadians(-150));
                    MecanumDrive drive = new MecanumDrive(hardwareMap, intPose1);
                    TrajectoryActionBuilder intStep1 = drive.actionBuilder(intPose1)
                            .strafeToLinearHeading(new Vector2d(-24-(8*Math.cos(Math.toRadians(30))),24-(8*Math.sin(Math.toRadians(30)))), Math.toRadians(-150));
                    TrajectoryActionBuilder intStep2 = drive.actionBuilder(intPose2)
                            .strafeToLinearHeading(new Vector2d(-24-(6.5*Math.cos(Math.toRadians(30))),24-(6.5*Math.sin(Math.toRadians(30)))), Math.toRadians(-150));

                    Actions.runBlocking(intStep1.build());
                    sleep(500);
                    servo.ToggleClamp();
                    Actions.runBlocking(intStep2.build());
                }
                else { servo.ToggleClamp(); }
                return false;
            }
        }
        public Action checkDistance() { return new CheckDistance(); }
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
        Sensor sensor = new Sensor();

        Pose2d firstPose = new Pose2d(-63, 24, Math.toRadians(0));
        Pose2d secondPose = new Pose2d(-61,0,Math.toRadians(0));
        Pose2d thirdPose = new Pose2d(-24-(8*Math.cos(Math.toRadians(30))),23-(8*Math.sin(Math.toRadians(30))),Math.toRadians(-150));
        Pose2d fourthPose = new Pose2d(-48, 24, Math.toRadians(45));


        MecanumDrive drive = new MecanumDrive(hardwareMap, firstPose);

        TrajectoryActionBuilder step1 = drive.actionBuilder(firstPose)
                .lineToX(-60)
                .strafeToConstantHeading(new Vector2d(-60, 0))
                .waitSeconds(0.1)
                .lineToX(-62);
        TrajectoryActionBuilder step2 = drive.actionBuilder(secondPose)
                .lineToX(-55)
                .turnTo(Math.toRadians(-150))
                .strafeToLinearHeading(new Vector2d(-24-(8*Math.cos(Math.toRadians(30))),23-(8*Math.sin(Math.toRadians(30)))),Math.toRadians(-150))
                .waitSeconds(0.5);
        TrajectoryActionBuilder step3 = drive.actionBuilder(thirdPose)
                .strafeToLinearHeading(new Vector2d(-48,24), Math.toRadians(45));
        TrajectoryActionBuilder step4 = drive.actionBuilder(fourthPose)
                .lineToY(36)
                .lineToY(54, new TranslationalVelConstraint(10))
                .waitSeconds(0.5)
                .lineToY(48, new TranslationalVelConstraint(10));


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

                        //sensor.checkDistance(),

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
