package edu.edina.library.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotState {
    public CurrentOperation CurrentOperation;

    public DriveSpeed DriveSpeed = edu.edina.library.util.DriveSpeed.Medium;

    public long SlideMotorLocation = 0;
    public SlideMotorAction SlideMotorAction = edu.edina.library.util.SlideMotorAction.Idle;
    public boolean AutoFoldInArm = false;
    public boolean AutoFoldOutArm = false;
    public boolean IntakeClampOpen;
    public double FlipPosition = 0.45;
    public boolean SlideSwitch = false;

    public long LiftDiff;
    public long LiftMotorLocation = 0;
    public ClawServoPosition ClawServoPosition = edu.edina.library.util.ClawServoPosition.Closed;
    public ElbowServoPosition ElbowServoPosition = edu.edina.library.util.ElbowServoPosition.In;
    public LiftFilpServoPosition LiftFilpServoPosition = edu.edina.library.util.LiftFilpServoPosition.Middle;
    public PoleLocation TargetPoleLocation = edu.edina.library.util.PoleLocation.None;
    public double ClawPosition = 0.0;
    public double ElbowPosition = 0.0;
    public double LiftFlipPosition = 0.0;
    public boolean LiftSwitch = false;
    public boolean LiftReadyForCone = false;

    public boolean IntakeSuccessfullySetup = false;
    public boolean LiftSuccessfullySetup = false;
    public boolean DriveSuccessfullySetup = false;

    public RobotState() {}

    public void telemetry(Telemetry telemetry) {
        if (IntakeSuccessfullySetup) {
            telemetry.addData("Slide Position", SlideMotorLocation);
            telemetry.addData("Folding Arm In", AutoFoldInArm);
            telemetry.addData("IntakeClampOpen", IntakeClampOpen);
            telemetry.addData("FlipPosition", FlipPosition);
            telemetry.addData("SlideSwitch", SlideSwitch);
        } else {
            telemetry.addData("Unable to setup motors slideMotor or flipMotor or setup servos flipServo or intakeServo", "");
        }

        if (LiftSuccessfullySetup) {
            telemetry.addData("Lift Position", LiftMotorLocation);
            telemetry.addData("ClawPosition", ClawPosition);
            telemetry.addData("ElbowPosition", ElbowPosition);
            telemetry.addData("LiftFlipPosition", LiftFlipPosition);
            telemetry.addData("LiftDiff", LiftDiff);
            telemetry.addData("LiftSwitch", LiftSwitch);
            telemetry.addData("LiftReadyForCone", LiftReadyForCone);
        } else {
            telemetry.addData("Unable to setup motors liftMotor or setup servos armServo or latchServo", "");
        }

        if (DriveSuccessfullySetup) {
            telemetry.addData("Drive Speed", DriveSpeed);
        } else {
            telemetry.addData("edu.edina.library.subsystems.MecanumDrive: Unable to setup frontLeft, frontRight, backLeft, backRight motors", "");
        }
    }
}
