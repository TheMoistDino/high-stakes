package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(540);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        double a = 6.5;
        /*
        Pose2d firstPose = new Pose2d(-52, -24, Math.toRadians(180));
        Pose2d secondPose = new Pose2d(-32, -24, Math.toRadians(180));
        Pose2d thirdPose = new Pose2d(-48, -24, Math.toRadians(-45));
        Pose2d fourthPose = new Pose2d(-18, -54, Math.toRadians(-45));
        Pose2d fifthPose = new Pose2d(-60, -44, Math.toRadians(-90));

        TrajectoryActionBuilder step1 = myBot.getDrive().actionBuilder(firstPose)
                .lineToX(-32)
                .waitSeconds(0.3);
        TrajectoryActionBuilder step2 = myBot.getDrive().actionBuilder(secondPose)
                .lineToXLinearHeading(-48, Math.toRadians(-45));
        TrajectoryActionBuilder step3 = myBot.getDrive().actionBuilder(thirdPose)
                .lineToY(-36)
                .lineToY(-54, new TranslationalVelConstraint(10));
        TrajectoryActionBuilder step4 = myBot.getDrive().actionBuilder(fourthPose)
                .strafeToLinearHeading(new Vector2d(-60, -44), Math.toRadians(-90));
        TrajectoryActionBuilder step5 = myBot.getDrive().actionBuilder(fifthPose)
                .lineToY(-52, new TranslationalVelConstraint(10))
                .strafeTo(new Vector2d(-30, -52), new TranslationalVelConstraint(15));


        myBot.runAction(
                new SequentialAction
                (
                        step1.build(),
                        new SleepAction(0.2),
                        step2.build(),
                        new SleepAction(0.2),
                        step3.build(),
                        new SleepAction(2.0),
                        step4.build(),
                        new SleepAction(0.2),
                        step5.build()
                )
        );
         */

        Pose2d firstPose = new Pose2d(-62, -24, Math.toRadians(0));
        Pose2d secondPose = new Pose2d(-61,0,Math.toRadians(0));
        Pose2d thirdPose = new Pose2d(-28,-21,Math.toRadians(-210));
        Pose2d fourthPose = new Pose2d(-48, -24, Math.toRadians(-45));

        TrajectoryActionBuilder step1 = myBot.getDrive().actionBuilder(firstPose)
                .lineToX(-60)
                .strafeToConstantHeading(new Vector2d(-60, 0))
                .waitSeconds(0.1)
                .lineToX(-61);
        TrajectoryActionBuilder step2 = myBot.getDrive().actionBuilder(secondPose)
                .lineToX(-55)
                .turnTo(Math.toRadians(-210))
                .strafeToLinearHeading(new Vector2d(-28,-21),Math.toRadians(-210))
                .waitSeconds(0.5);
        TrajectoryActionBuilder step3 = myBot.getDrive().actionBuilder(thirdPose)
                .strafeToLinearHeading(new Vector2d(-48,-24), Math.toRadians(-45));
        TrajectoryActionBuilder step4 = myBot.getDrive().actionBuilder(fourthPose)
                .lineToY(-36)
                .lineToY(-54, new TranslationalVelConstraint(10));

        myBot.runAction(
                new SequentialAction(
                        step1.build(),
                        new SleepAction(0.5),
                        step2.build(),
                        step3.build(),
                        step4.build(),
                        new SleepAction(2.0)
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}