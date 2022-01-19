package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity red = new DefaultBotBuilder(meepMeep)
            .setColorScheme(new ColorSchemeBlueDark())
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(start ->
                /*        start.trajectorySequenceBuilder(new Pose2d(-32, -65, Math.toRadians(90)))
                //duck
                .splineToLinearHeading(new Pose2d(-48, -44, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56,-48, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(1)
                //deposit preloaded, adjust by 2" for low/mid/high
                .lineToConstantHeading(new Vector2d(-32, -24))
                .addDisplacementMarker(88,()->{})
                //intake duck
                .splineToLinearHeading(new Pose2d(-40, -56, Math.toRadians(240)), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-48, -56))
                //deposit duck
                .lineToLinearHeading(new Pose2d(-32, -24, Math.toRadians(180)))
                .build()*/

                        start.trajectorySequenceBuilder(new Pose2d(12, -63, Math.toRadians(270)))
                                //deposit preloaded, adjust by 2" for low/mid/high
                                .splineToLinearHeading(new Pose2d(2, -38, Math.toRadians(315)), Math.toRadians(135))
                                //intake1
                                .splineToLinearHeading(new Pose2d(12, -63, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(48, -63, Math.toRadians(0)))
                                .waitSeconds(1)
                                //deposit1
                                .lineToSplineHeading(new Pose2d(12, -63, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(2, -38, Math.toRadians(315)), Math.toRadians(135)) //deposit
                                //looop after
                                .build()
                );

        RoadRunnerBotEntity blue = new DefaultBotBuilder(meepMeep)
            .setColorScheme(new ColorSchemeRedDark())
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(start ->
                        /*start.trajectorySequenceBuilder(new Pose2d(-32, 65, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-48, 44, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-56,48, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(1)
                                //deposit preloaded, adjust by 2" for low/mid/high
                                .lineToConstantHeading(new Vector2d(-32, 24))
                                .addDisplacementMarker(88,()->{})
                                //intake duck
                                .splineToLinearHeading(new Pose2d(-40, 56, Math.toRadians(120)), Math.toRadians(180))
                                .lineToConstantHeading(new Vector2d(-48, 56))
                                //deposit duck
                                .lineToLinearHeading(new Pose2d(-32, 24, Math.toRadians(180)))
                                .build()*/

                start.trajectorySequenceBuilder(new Pose2d(12, 63, Math.toRadians(270)))
                        //deposit preloaded, adjust by 2" for low/mid/high
                        .splineToLinearHeading(new Pose2d(2, 38, Math.toRadians(45)), Math.toRadians(270))
                        //intake1
                        .splineToLinearHeading(new Pose2d(12, 63, Math.toRadians(0)), Math.toRadians(0))
                        .lineToLinearHeading(new Pose2d(48, 63, Math.toRadians(0)))
                        .waitSeconds(1)
                        //deposit1
                        .lineToSplineHeading(new Pose2d(12, 63, Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(2, 38, Math.toRadians(45)), Math.toRadians(225)) //deposit
                        //looop after
                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(red)
                .addEntity(blue)
                .start();
    }
}