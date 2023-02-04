package edu.edina.opmodes.auto;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Vector;

import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.RobotState;
import edu.edina.library.vision.AprilTagDetectionPipeline;

@Autonomous
@Config
//@Disabled
public class RightSideMedium extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double dropTime = .5;
    double pickupTime = .5;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int detectionId = 6; // middle

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public static double PICKUPY = -8.5;

    private DcMotorEx liftMotor;
    private RobotState robotState;
    private Servo armServo;
    private Servo clawServo;
    private ColorSensor backColor;
    private DistanceSensor frontDistance;

    @Override
    public void runOpMode() {
/*        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
*/
        double dropoffX1 = 23;
        double dropoffX2 = 25;
        double dropoffX3 = 25;
        double dropoffX4 = 25;
        double dropoffX5 = 25;
        double dropoffX6 = 25;
        double dropoffY = -6;
        double pickupX = 58;

        robotState = new RobotState();
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");
        frontDistance = hardwareMap.get(DistanceSensor.class, "frontDistance");
        final float[] hsvValues = new float[3];

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setPower(1);
        robotState.FutureTargetPosition = 0;
        liftMotor.setTargetPosition(robotState.FutureTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        robotState.ClawServoPosition = ClawServoPosition.Closed;

        armServo.setPosition(robotState.ARMFRONTPOSITION);
        robotState.ArmServoPosition = ArmServoPosition.Front;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(35, -65, Math.toRadians(-180)));

        // cone one drop off
        TrajectorySequence start = drive.trajectorySequenceBuilder(new Pose2d(33, -65, Math.toRadians(-180)))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);})
                .addTemporalMarker(1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITION); })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .strafeTo(new Vector2d(32, -23))
                .build();

        // cone two pickup
        TrajectorySequence backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeRight(12)
                .back(23)
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION5);
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .build();

        TrajectorySequence strafe = drive.trajectorySequenceBuilder(backToPickup1.end())
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 10;
                    }
                })
                .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 10;
                    }
                })
                .strafeRight(5).build();

        // cone two dropoff
        TrajectorySequence backToDropOff1 = drive.trajectorySequenceBuilder(new Pose2d(60, -8, Math.toRadians(-180)))
                .resetConstraints()
                .strafeTo(new Vector2d(24, -12))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITION);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                    } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone three pickup
        TrajectorySequence backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeTo(new Vector2d(60, -8))
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION4);
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone three dropoff
        TrajectorySequence backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .strafeTo(new Vector2d(24, -12))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITION);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone four pickup
        TrajectorySequence backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .strafeTo(new Vector2d(60, -8))
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION3);
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone four drop off
        TrajectorySequence backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .strafeTo(new Vector2d(24, -12))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITION);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // cone five pickup
        TrajectorySequence backToPickup4 = drive.trajectorySequenceBuilder(backToDropOff3.end())
                .strafeTo(new Vector2d(60, -8))
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION2);
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone five drop off
        TrajectorySequence backToDropOff4 = drive.trajectorySequenceBuilder(backToPickup4.end())
                .strafeTo(new Vector2d(24, -12))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITION);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();


        // cone six pickup
        TrajectorySequence backToPickup5 = drive.trajectorySequenceBuilder(backToDropOff4.end())
                .strafeTo(new Vector2d(60, -8))
                .addTemporalMarker(.7, () -> {
                    armServo.setPosition(robotState.ARMBACKPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Back;
                })
                .addTemporalMarker(1.0, () -> {
                    liftMotor.setTargetPosition(robotState.CONESTACKPOSITION1);
                })
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                } )
                .build();

        // cone six drop off
        TrajectorySequence backToDropOff5 = drive.trajectorySequenceBuilder(backToPickup5.end())
                .strafeTo(new Vector2d(24, -12))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.AUTOPOLEPOSITION);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMSIDEPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Side;
                } )
                .addTemporalMarker(2, () -> {
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                } )
                .build();

        // park
        TrajectorySequence backToPickup6 = drive.trajectorySequenceBuilder(backToDropOff5.end())
                .addTemporalMarker(0.1, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                })
                .addTemporalMarker(0.4, () -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(35)
                .build();

        while (!isStarted()) {
            telemetry.addData("Start Duration", start.duration());
            telemetry.addData("Back to Pickup1 Duration", backToPickup1.duration());
            telemetry.addData("Back to Drop off1 Duration", backToDropOff1.duration());
            telemetry.addData("Back to Pickup2 Duration", backToPickup2.duration());
            telemetry.addData("Back to Drop off2 Duration", backToDropOff2.duration());
            telemetry.addData("Back to Pickup3 Duration", backToPickup3.duration());
            telemetry.addData("Back to Drop off3 Duration", backToDropOff3.duration());
            telemetry.update();
        }

        if (opModeIsActive()) {
            liftMotor.setTargetPosition(-50);
            liftMotor.setPower(1);
            drive.followTrajectorySequence(start);

            drive.followTrajectorySequence(backToPickup1);

            sleep(100);

            drive.followTrajectorySequenceAsync(strafe);
            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                Color.RGBToHSV(backColor.red() * 8, backColor.green() * 8, backColor.blue() * 8, hsvValues);
                if ((hsvValues[0] < 100) || (hsvValues[0] > 180)){
                    drive.breakFollowing();
                    drive.setDrivePower(new Pose2d());
                    break;
                }

                drive.update();
                idle();
            }

            TrajectorySequence left = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setVelConstraint(new TrajectoryVelocityConstraint() {
                        @Override
                        public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                            return 10;
                        }
                    })
                    .setAccelConstraint(new TrajectoryAccelerationConstraint() {
                        @Override
                        public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                            return 10;
                        }
                    })
                    .strafeLeft(1)
                    .build();

            //drive.followTrajectorySequence(left);

            double distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH) - 1;
            if (distanceToTravel > 4) {
                distanceToTravel = 4;
            }

            TrajectorySequence back = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .back(distanceToTravel)
                    .build();

            drive.followTrajectorySequence(back);
/*
            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            sleep(200);

            drive.setPoseEstimate(new Pose2d(60, -8, Math.toRadians(-180)));

            drive.followTrajectorySequence(backToDropOff1);

            drive.followTrajectorySequence(backToPickup2);

            drive.followTrajectorySequence(backToDropOff2);

            drive.followTrajectorySequence(backToPickup3);

            drive.followTrajectorySequence(backToDropOff3);

            drive.followTrajectorySequence(backToPickup4);

            drive.followTrajectorySequence(backToDropOff4);

            drive.followTrajectorySequence(backToPickup5);

            drive.followTrajectorySequence(backToDropOff5);

            drive.followTrajectorySequence(backToPickup6);
*/
            while (opModeIsActive()) {
                distanceToTravel = frontDistance.getDistance(DistanceUnit.INCH);
                telemetry.addData("distanceToTravel", distanceToTravel);
                telemetry.update();
                sleep(20);
            }
        }
    }
}
