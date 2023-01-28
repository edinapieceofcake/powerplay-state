package edu.edina.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.RobotState;
import edu.edina.library.vision.AprilTagDetectionPipeline;

@Autonomous
@Config
//@Disabled
public class RightSideLow extends LinearOpMode {

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
    private DigitalChannel liftSwitch;

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
        liftSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");

        liftSwitch.setMode(DigitalChannel.Mode.INPUT);

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
        // cone one
        TrajectorySequence start = drive.trajectorySequenceBuilder(new Pose2d(33, -65, Math.toRadians(-180)))
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONLOW);})
                .addTemporalMarker(1, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONMIDDLE + 25); })
                .strafeTo(new Vector2d(32, -23))
                .build();

        TrajectorySequence backToPickup1 = drive.trajectorySequenceBuilder(start.end())
                .strafeRight(15)
                .back(25)
                .build();

        // cone two
        TrajectorySequence backToDropOff1 = drive.trajectorySequenceBuilder(backToPickup1.end())
//                .forward(10)
//                .splineToConstantHeading(new Vector2d(34, -23), Math.toRadians(-180))
                .forward(26)
                .strafeLeft(15.5)
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONMIDDLE + 25);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                } )
                //.forward(2)
                .build();

        TrajectorySequence backToPickup2 = drive.trajectorySequenceBuilder(backToDropOff1.end())
                .strafeRight(15)
                .back(25)
                .build();

        // cone three
        TrajectorySequence backToDropOff2 = drive.trajectorySequenceBuilder(backToPickup2.end())
                .forward(10)
                .splineToConstantHeading(new Vector2d(33, -26), Math.toRadians(-180))
//                        .forward(23)
//                        .strafeLeft(15)
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONMIDDLE + 15);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                } )
                //.forward(2)
                .build();

        TrajectorySequence backToPickup3 = drive.trajectorySequenceBuilder(backToDropOff2.end())
                .strafeRight(14)
                .back(25)
                .build();

        // cone foue
        TrajectorySequence backToDropOff3 = drive.trajectorySequenceBuilder(backToPickup3.end())
                .forward(10)
                .splineToConstantHeading(new Vector2d(36, -28), Math.toRadians(-180))
//                        .forward(23)
//                        .strafeLeft(15)
                .addTemporalMarker(.1, () -> { liftMotor.setTargetPosition(robotState.POLEPOSITIONMIDDLE + 25);})
                .addTemporalMarker(.5, () -> {
                    armServo.setPosition(robotState.ARMFRONTPOSITION);
                    robotState.ArmServoPosition = ArmServoPosition.Front;
                } )
                //.forward(2)
                .build();

        TrajectorySequence backToPickup4 = drive.trajectorySequenceBuilder(backToDropOff3.end())
                .strafeRight(15)
                .back(25)
                .build();

        waitForStart();

        if (opModeIsActive()) {
            liftMotor.setTargetPosition(-50);
            liftMotor.setPower(1);
            drive.followTrajectorySequence(start);

            /*
            liftMotor.setTargetPosition(robotState.POLEPOSITIONMIDDLE);

            while (opModeIsActive() && (liftMotor.getCurrentPosition() - robotState.POLEPOSITIONMIDDLE > 10))
                idle();

            armServo.setPosition(robotState.ARMSIDEPOSITION - .03);
            robotState.ArmServoPosition = ArmServoPosition.Side;

            sleep(750);

            liftMotor.setTargetPosition(robotState.POLEPOSITIONMIDDLE + 10);
*/
            //drive.followTrajectorySequence(forward);

            clawServo.setPosition(robotState.CLAWOPENPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Open;

            sleep(200);

            armServo.setPosition(robotState.ARMBACKPOSITION);
            robotState.ArmServoPosition = ArmServoPosition.Back;

            sleep(400);

            liftMotor.setTargetPosition(-170);

            //drive.setPoseEstimate(new Pose2d(32, -23, Math.toRadians(-180)));

            drive.followTrajectorySequence(backToPickup1);

            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            sleep(200);

            drive.followTrajectorySequence(backToDropOff1);

            clawServo.setPosition(robotState.CLAWOPENPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Open;

            sleep(200);

            armServo.setPosition(robotState.ARMBACKPOSITION);
            robotState.ArmServoPosition = ArmServoPosition.Back;

            sleep(400);

            liftMotor.setTargetPosition(-128);

            drive.followTrajectorySequence(backToPickup2);

            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            sleep(300);

            drive.followTrajectorySequence(backToDropOff2);

            clawServo.setPosition(robotState.CLAWOPENPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Open;

            sleep(300);

            armServo.setPosition(robotState.ARMBACKPOSITION);
            robotState.ArmServoPosition = ArmServoPosition.Back;

            sleep(400);

            liftMotor.setTargetPosition(-86);

            drive.followTrajectorySequence(backToPickup3);

            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            sleep(300);

            drive.followTrajectorySequence(backToDropOff3);

            clawServo.setPosition(robotState.CLAWOPENPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Open;

            sleep(300);

            liftMotor.setTargetPosition(0);// -53


            drive.followTrajectorySequence(backToPickup4);

            while (opModeIsActive()) {
                sleep(20);
            }
        }
    }
}
