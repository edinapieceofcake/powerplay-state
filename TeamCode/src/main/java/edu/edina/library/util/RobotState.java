package edu.edina.library.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotState {
    public DriveSpeed DriveSpeed = edu.edina.library.util.DriveSpeed.Medium;

    public long LiftDiff;
    public long LiftMotorLocation = 0;
    public ClawServoPosition ClawServoPosition = edu.edina.library.util.ClawServoPosition.Closed;
    public ArmServoPosition ArmServoPosition = edu.edina.library.util.ArmServoPosition.Front;
    public PoleLocation TargetPoleLocation = edu.edina.library.util.PoleLocation.None;
    public double ClawPosition = 0.0;
    public double ArmPosition = 0.0;
    public boolean LiftSwitch = false;
    public boolean LiftMotorReset = false;
    public double SpeedMultiplier = 0.0;
    public double CLAWOPENPOSITION = 0.56;
    public double CLAWCLOSEDPOSITION = 0.83;
    public int CLAWOPENPOSITION100 = 56;
    public double ARMFRONTPOSITION = 0.14;
    public double ARMBACKPOSTITION = 0.83;
    public double ARMSIDEPOSITION = 0.5;

    public int POLEPOSITIONLOW = -1200;
    public int POLEPOSITIONMIDDLE = -2060;
    public int POLEPOSITIONHIGH = -2850;
    public int LIFTWAITTIME = 250;

    public int CLAWOPENWAITTIME = 250;
    public int LIFTRETURNHEiGHT = 0;
    public boolean LiftSuccessfullySetup = false;
    public boolean DriveSuccessfullySetup = false;
    public int FutureTargetPosition = 0;

    public double LowSpeedMultiplier = .5;
    public double HighSpeedMultiplier = .6;

    public RobotState() {}

    public void telemetry(Telemetry telemetry) {
        if (LiftSuccessfullySetup) {
            telemetry.addData("Lift Position", LiftMotorLocation);
            telemetry.addData("ClawPosition", ClawPosition);
            telemetry.addData("ArmPosition", ArmPosition);
            telemetry.addData("LiftDiff", LiftDiff);
            telemetry.addData("LiftSwitch", LiftSwitch);
            telemetry.addData("LiftMotorReset", LiftMotorReset);
            telemetry.addData("Future Target Position", FutureTargetPosition);
            telemetry.addData("TargetPoleLocation", TargetPoleLocation);
        } else {
            telemetry.addData("Unable to setup motors liftMotor or setup servos armServo or latchServo", "");
        }

        if (DriveSuccessfullySetup) {
            telemetry.addData("Drive Speed", DriveSpeed);
            telemetry.addData("Speed Multiplier", SpeedMultiplier);
            telemetry.addData("LowSpeedMultiplier", LowSpeedMultiplier);
            telemetry.addData("HighSpeedMultiplier", HighSpeedMultiplier);
        } else {
            telemetry.addData("edu.edina.library.subsystems.MecanumDrive: Unable to setup frontLeft, frontRight, backLeft, backRight motors", "");
        }
    }
}
