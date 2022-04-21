 package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;
import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity red = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(start ->
                                start.trajectorySequenceBuilder(new Pose2d(-36, 62, Math.toRadians(180)))
                                        //high
                                        .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(225)), Math.toRadians(180))
                                        /*mid / low
                                        .splineToLinearHeading(new Pose2d(-24, 40, Math.toRadians(45)), Math.toRadians(180))*/
                                        .splineToLinearHeading(new Pose2d(-58, 58, Math.toRadians(270)), Math.toRadians(90))
                                        .lineToConstantHeading(new Vector2d(-50, 60))
                                        .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(225)), Math.toRadians(180))
                                        .splineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(180)), Math.toRadians(180))


                                        //.lineToConstantHeading(new Vector2d(44, -62))
                                        //.lineToConstantHeading(new Vector2d(-12, -62))
                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(red)
                .start();
    }
}