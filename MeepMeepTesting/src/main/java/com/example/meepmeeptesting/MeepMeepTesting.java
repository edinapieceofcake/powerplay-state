package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double dropTime = .8;
        double pickupTime = 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 12)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -60, Math.toRadians(90)))
                                .forward(45)
                                .splineTo(new Vector2d(-35,-10), Math.toRadians(45))
                                .build()
                );
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 12)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -10, Math.toRadians(45)))
                                .back(10)
                                .splineTo(new Vector2d(-58,-10), Math.toRadians(180))
                                .build()
                );
        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 12)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-58, -10, Math.toRadians(0)))
                                .forward(1)
                                .splineTo(new Vector2d(-35,-10), Math.toRadians(45))
                                .build()
                );

        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12, 12)
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(-90)))
                                .forward(25)
                                .splineTo(new Vector2d(-25,10), Math.toRadians(0))
                                .build()
                );
        RoadRunnerBotEntity myBot5 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(12, 12)
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-25, 10, Math.toRadians(0)))
                                .strafeTo(new Vector2d(-58,12))
                                .build()
                );
        RoadRunnerBotEntity myBot6 = new DefaultBotBuilder(meepMeep)
                .setDimensions(12, 12)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-58, 12, Math.toRadians(0)))
                                .strafeTo(new Vector2d(-25,10))
                                .build()
                );

        RoadRunnerBotEntity myBot7 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12, 12)
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(-90)))
                                .forward(30)
                                .splineTo(new Vector2d(25,10), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity myBot8 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12, 12)
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(25, 10, Math.toRadians(0)))
                                .strafeTo(new Vector2d(58,12))
                                .build()
                );

        RoadRunnerBotEntity myBot9 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12, 12)
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(58, 12, Math.toRadians(0)))
                                .strafeTo(new Vector2d(25,10))
                                .build()
                );

        // this bot does the entire path for the right side red. the drop time is .8 second and one second to pickup
        RoadRunnerBotEntity myBot10 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(12, 12)
                .setConstraints(55, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                                .forward(30)
                                .splineTo(new Vector2d(25,-10), Math.toRadians(180))
                                .waitSeconds(dropTime) // drop first cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab second cone
                                .strafeTo(new Vector2d(25, -10))
                                .waitSeconds(dropTime) // drop second cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab third cone
                                .strafeTo(new Vector2d(25, -10))
                                .waitSeconds(dropTime) // drop third cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab fourth cone
                                .strafeTo(new Vector2d(25, -10))
                                .waitSeconds(dropTime) // drop fourth cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab fifth cone
                                .strafeTo(new Vector2d(25, -10))
                                .waitSeconds(dropTime) // drop fifth cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab sixth cone
                                .strafeTo(new Vector2d(25, -10))
                                .waitSeconds(dropTime) // drop sixth cone
                                .strafeTo(new Vector2d(50, -12)) // park
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .addEntity(myBot4)
                .addEntity(myBot5)
                .addEntity(myBot6)
                .addEntity(myBot7)
                .addEntity(myBot8)
                .addEntity(myBot9)
                // comment this bot out if you want ot see all the other paths run faster as this bot is taking the entire 30 seconds.
                .addEntity(myBot10)
                .start();
    }
}