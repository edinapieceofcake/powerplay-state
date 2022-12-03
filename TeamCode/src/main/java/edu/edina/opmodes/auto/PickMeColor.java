/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package edu.edina.opmodes.auto;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class PickMeColor extends LinearOpMode
{
    NormalizedColorSensor colorSensor;

    View relativeLayout;

    @Override
    public void runOpMode()
    {
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        Servo liftFlipServo = hardwareMap.get(Servo.class, "liftFlipServo");
        Servo elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo clampServo = hardwareMap.get(Servo.class, "clampServo");
        Servo armFlipServo = hardwareMap.get(Servo.class, "armFlipServo");
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = null;

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(.55);
        elbowServo.setPosition(.6);
        liftFlipServo.setPosition(.6);
        clampServo.setPosition(.5);
        armFlipServo.setPosition(.45);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        colorSensor.setGain(2);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .back(16)
                .build();

        drive.followTrajectorySequence(trajectory);

        // Get the normalized colors from the sensor
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startTime + 5000) {
            colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Time", (System.currentTimeMillis() - startTime))
                    .addData("Hue", "%.3f", hsvValues[0]);

            telemetry.update();
        }

        if (hsvValues[0] < 170.0) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .back(11)
                    .strafeRight(23)
                    .build();
        } else if (hsvValues[0] > 290) {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .back(11)
                    .strafeLeft(23)
                    .build();
        } else {
            trajectory = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                    .back(11)
                    .build();
        }

        drive.followTrajectorySequence(trajectory);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

        while (opModeIsActive()) {sleep(20);}
    }
}