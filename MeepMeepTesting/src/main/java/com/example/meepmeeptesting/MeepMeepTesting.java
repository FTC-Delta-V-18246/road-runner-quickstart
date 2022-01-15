package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity path1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                /*.followTrajectorySequence(redDuck ->
                        redDuck.trajectorySequenceBuilder(new Pose2d(-30, -65, Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d(-52, -56, Math.toRadians(180)), Math.toRadians(180)) //duck
                                .splineToLinearHeading(new Pose2d(-32, -24, Math.toRadians(180)), Math.toRadians(0)) //deposit preload
                                .splineToLinearHeading(new Pose2d(-48, -56, Math.toRadians(240)), Math.toRadians(180)) //intake duck
                                .splineToLinearHeading(new Pose2d(-34, -24, Math.toRadians(180)), Math.toRadians(0)) //deposit duck
                                .build()
                );
                .followTrajectorySequence(blueDuck ->
                blueDuck.trajectorySequenceBuilder(new Pose2d(-30, 65, Math.toRadians(270)))
                        .splineToLinearHeading(new Pose2d(-52, 56, Math.toRadians(180)), Math.toRadians(180)) //duck
                        .splineToLinearHeading(new Pose2d(-32, 24, Math.toRadians(180)), Math.toRadians(0)) //deposit preload
                        .splineToLinearHeading(new Pose2d(-48, 56, Math.toRadians(120)), Math.toRadians(180)) //intake duck
                        .splineToLinearHeading(new Pose2d(-34, 24, Math.toRadians(180)), Math.toRadians(0)) //deposit duck
                        .build()
        );*/
                .followTrajectorySequence(blueCycle ->
                blueCycle.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                        .splineToLinearHeading(new Pose2d(-12, 45, Math.toRadians(90)), Math.toRadians(270)) //deposit
                        .splineToLinearHeading(new Pose2d(12, 63, Math.toRadians(0)), Math.toRadians(0)) //intake
                        .lineToLinearHeading(new Pose2d(48, 63, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(12, 63, Math.toRadians(0)))
                        .splineToLinearHeading(new Pose2d(-12, 45, Math.toRadians(90)), Math.toRadians(270)) //deposit

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(path1)
                .start();
    }
}