package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d firstPose = new Pose2d(-52, -24, Math.toRadians(-180));
        Pose2d secondPose = new Pose2d(-32, -24, Math.toRadians(-180));
        Pose2d thirdPose = new Pose2d(-48, -24, Math.toRadians(-45));

        TrajectoryActionBuilder step1 = myBot.getDrive().actionBuilder(firstPose)
                .lineToX(-32)
                .waitSeconds(0.3);
        TrajectoryActionBuilder step2 = myBot.getDrive().actionBuilder(secondPose)
                .lineToXLinearHeading(-48, Math.toRadians(-45));
        TrajectoryActionBuilder step3 = myBot.getDrive().actionBuilder(thirdPose)
                .lineToY(-36)
                .lineToY(-45, new TranslationalVelConstraint(10));

        myBot.runAction(
                new SequentialAction
                (
                        step1.build(),
                        new SleepAction(0.2),
                        step2.build(),
                        new SleepAction(0.2),
                        step3.build()
                )
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}