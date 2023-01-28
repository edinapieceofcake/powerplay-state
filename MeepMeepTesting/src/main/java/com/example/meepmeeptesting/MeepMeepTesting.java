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
        double dropTime = .7;
        double pickupTime = .9;

        // this bot does the entire path for the right side red. the drop time is .8 second and one second to pickup
        RoadRunnerBotEntity myBotLeft = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 40, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-59, -57), Math.toRadians(90))
                                .forward(40)
                                .splineTo(new Vector2d(-22, -8), Math.toRadians(0))
                                .waitSeconds(dropTime) // drop first cone
                                .strafeTo(new Vector2d(-58, -12))
                                .waitSeconds(pickupTime) // grab second cone
                                .strafeTo(new Vector2d(-22, -8))
                                .waitSeconds(dropTime) // drop second cone
                                .strafeTo(new Vector2d(-58, -12))
                                .waitSeconds(pickupTime) // grab third cone
                                .strafeTo(new Vector2d(-22, -8))
                                .waitSeconds(dropTime) // drop third cone
                                .strafeTo(new Vector2d(-58, -12))
                                .waitSeconds(pickupTime) // grab fourth cone
                                .strafeTo(new Vector2d(-22, -8))
                                .waitSeconds(dropTime) // drop fourth cone
                                .strafeTo(new Vector2d(-58, -12))
                                .waitSeconds(pickupTime) // grab fifth cone
                                .strafeTo(new Vector2d(-22, -8))
                                .waitSeconds(dropTime) // drop fifth cone
                                .strafeTo(new Vector2d(-58, -12))
                                .waitSeconds(pickupTime) // grab sixth cone
                                .strafeTo(new Vector2d(-22, -8))
                                .waitSeconds(dropTime) // drop sixth cone
                                .strafeTo(new Vector2d(-50, -12)) // park
                                .build()
                );

        RoadRunnerBotEntity myBotLeft2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 40, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -65, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(-11, -57), Math.toRadians(90))
                                .forward(40)
                                .splineTo(new Vector2d(-22, -8), Math.toRadians(0))
                                .waitSeconds(dropTime) // drop first cone
                                .strafeTo(new Vector2d(-58, -12))
                                .build()
                );

        RoadRunnerBotEntity myBotRight = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -65, Math.toRadians(90)))
                                .splineToConstantHeading(new Vector2d(11, -57), Math.toRadians(90))
                                .forward(40)
                                .splineTo(new Vector2d(22, -8), Math.toRadians(0))
                                .waitSeconds(dropTime) // drop first cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab second cone
                                .strafeTo(new Vector2d(22, -8))
                                .waitSeconds(dropTime) // drop second cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab third cone
                                .strafeTo(new Vector2d(22, -8))
                                .waitSeconds(dropTime) // drop third cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab fourth cone
                                .strafeTo(new Vector2d(22, -8))
                                .waitSeconds(dropTime) // drop fourth cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab fifth cone
                                .strafeTo(new Vector2d(22, -8))
                                .waitSeconds(dropTime) // drop fifth cone
                                .strafeTo(new Vector2d(58, -12))
                                .waitSeconds(pickupTime) // grab sixth cone
                                .strafeTo(new Vector2d(22, -8))
                                .waitSeconds(dropTime) // drop sixth cone
                                .strafeTo(new Vector2d(50, -12)) // park
                                .build());

        RoadRunnerBotEntity myBotRight2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(35, -65, Math.toRadians(90)))
                                        .splineToConstantHeading(new Vector2d(59, -57), Math.toRadians(90))
                                        .forward(30)
                                        .splineTo(new Vector2d(22, -16), Math.toRadians(-180))
                                        .build());

        RoadRunnerBotEntity myBotRight3 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(9, 9)
                .setConstraints(35, 30, Math.toRadians(180), Math.toRadians(90), 11)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -65, Math.toRadians(-180)))
                                .strafeTo(new Vector2d(33, -23))
                                //.strafeRight(42)
                                .waitSeconds(.2)
                                .strafeTo(new Vector2d(43, -8))
//                                .splineTo(new Vector2d(45, -8), Math.toRadians(-180))
                                .back(10)
                                .forward(10)
                                .splineToConstantHeading(new Vector2d(33, -23), Math.toRadians(-180))
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(myBotLeft)
                //.addEntity(myBotLeft2)
                .addEntity(myBotRight3)
                .start();
    }
}