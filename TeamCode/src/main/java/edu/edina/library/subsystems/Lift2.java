package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.PoleLocation;
import edu.edina.library.util.RobotState;

public class Lift2 extends edu.edina.library.subsystems.Subsystem {

    private static double CLAWOPENPOSITION = 0.56;
    private static double CLAWCLOSEDPOSITION = 0.82;
    private static int CLAWOPENPOSITION100 = 56;

    private static double ARMFRONTPOSITION = 0.14;
    private static double ARMBACKPOSTITION = 0.83;
    private static double ARMSIDEPOSITION = 0.5;


    private static int POLEPOSITIONLOW = -1008;
    private static int POLEPOSITIONMIDDLE = -1900;
    private static int POLEPOSITIONHIGH = -2600;

    private static int CLAWOPENWAITTIME = 250;
    private static int LIFTRETURNHEiGHT = 0;

    private DcMotorEx liftMotor;
    private RobotState robotState;
    private Servo armServo;
    private Servo clawServo;
    private DigitalChannel liftSwitch;

    private double liftSpeed;
    private boolean runningToPosition;
    private boolean atZeroPosition;
    private int targetPosition = 0;
    private long clawOpenStartedTime = 0;
    private boolean liftMotorReset = false;
    private boolean clawOpen = false;

    public Lift2(HardwareMap map, RobotState robotState) {
        try {
            liftMotor = map.get(DcMotorEx.class, "liftMotor");
            armServo = map.get(Servo.class, "armServo");
            clawServo = map.get(Servo.class, "clawServo");
            liftSwitch = map.get(DigitalChannel.class, "liftSwitch");

            // set the digital channel to input.
            liftSwitch.setMode(DigitalChannel.Mode.INPUT);

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            clawServo.setPosition(CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            armServo.setPosition(ARMFRONTPOSITION);
            robotState.ArmServoPosition = ArmServoPosition.Front;

            robotState.TargetPoleLocation = PoleLocation.None;
            robotState.LiftSuccessfullySetup = true;
        } catch (Exception ex) {
            robotState.LiftSuccessfullySetup = false;
        }

        this.robotState = robotState;
    }

    @Override
    public void update() {
        if (robotState.TargetPoleLocation != PoleLocation.None) {
            if (robotState.TargetPoleLocation == PoleLocation.Return) {
                if (!runningToPosition) {
                    clawServo.setPosition(CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawOpenStartedTime = System.currentTimeMillis();
                    runningToPosition = true;
                    clawOpen = false;
                } else if (!clawOpen) {
                    if ((System.currentTimeMillis() > (clawOpenStartedTime + CLAWOPENWAITTIME)) && (Math.round(clawServo.getPosition() * 100) == CLAWOPENPOSITION100)) {
                        liftMotor.setTargetPosition(LIFTRETURNHEiGHT);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1);
                        clawOpen = true;
                        atZeroPosition = false;
                    }
                } else if (!atZeroPosition) {
                    robotState.LiftDiff = Math.abs(liftMotor.getCurrentPosition());

                    if (robotState.LiftDiff < Math.abs(LIFTRETURNHEiGHT)) {
                        resetState();
                    }
                }
            } else {
                if (!runningToPosition) {
                    if (robotState.TargetPoleLocation == PoleLocation.Low) {
                        targetPosition = POLEPOSITIONLOW;
                    } else if (robotState.TargetPoleLocation == PoleLocation.Medium) {
                        targetPosition = POLEPOSITIONMIDDLE;
                    } else if (robotState.TargetPoleLocation == PoleLocation.High) {
                        targetPosition = POLEPOSITIONHIGH;
                    }

                    clawServo.setPosition(CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                    liftMotor.setTargetPosition(targetPosition);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                    runningToPosition = true;
                } else {
                    if (robotState.ClawServoPosition == ClawServoPosition.Open) {
                        clawServo.setPosition(CLAWOPENPOSITION);
                    } else if (robotState.ClawServoPosition == ClawServoPosition.Closed) {
                        clawServo.setPosition(CLAWCLOSEDPOSITION);
                    }
                }
            }
        } else {
            if (robotState.ClawServoPosition == ClawServoPosition.Open) {
                clawServo.setPosition(CLAWOPENPOSITION);
            } else if (robotState.ClawServoPosition == ClawServoPosition.Closed) {
                clawServo.setPosition(CLAWCLOSEDPOSITION);
            }

            if (robotState.ArmServoPosition == ArmServoPosition.Front) {
                armServo.setPosition(ARMFRONTPOSITION);
            } else if (robotState.ArmServoPosition == ArmServoPosition.Side) {
                armServo.setPosition(ARMSIDEPOSITION);
            } else if (robotState.ArmServoPosition == ArmServoPosition.Back) {
                armServo.setPosition(ARMBACKPOSTITION);
            }

            if (liftSwitch.getState() && !liftMotorReset) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotorReset = true;
            } else {
                liftMotorReset = false;
            }

            liftMotor.setPower(liftSpeed);
        }

        robotState.LiftMotorLocation = liftMotor.getCurrentPosition();
        robotState.ClawPosition = Math.round(clawServo.getPosition() * 100);
        robotState.ArmPosition = Math.round(armServo.getPosition() * 100);
        robotState.LiftSwitch = liftSwitch.getState();
    }

    public void setLiftProperties(double liftUp, double liftDown, boolean armFront, boolean armSide, boolean armBack,
                                  boolean clawOpen, boolean clawClosed, boolean lowPole, boolean mediumPole,
                                  boolean highPole, boolean returnPosition) {

        if (robotState.TargetPoleLocation != PoleLocation.None) {
            if ((liftUp != 0) || (liftDown != 0)) {
                resetState();
            }
        }

        if (liftUp != 0)
            this.liftSpeed  = liftUp;
        else if (liftDown != 0)
            this.liftSpeed = -liftDown;
        else
            this.liftSpeed = 0;

        if (armFront) {
            robotState.ArmServoPosition = ArmServoPosition.Front;
        } else if (armSide) {
            robotState.ArmServoPosition = ArmServoPosition.Side;
        } else if (armBack) {
            robotState.ArmServoPosition = ArmServoPosition.Back;
        }

        if (clawOpen) {
            robotState.ClawServoPosition = ClawServoPosition.Open;
        } else if (clawClosed) {
            robotState.ClawServoPosition = ClawServoPosition.Closed;
        }
        if (lowPole) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.Low;
        } else if (mediumPole) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.Medium;
        } else if (highPole) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.High;
        } else if (returnPosition) {
            runningToPosition = false;
            robotState.TargetPoleLocation = PoleLocation.Return;
        }
    }

    private void resetState() {
        runningToPosition = false;
        atZeroPosition = false;
        robotState.TargetPoleLocation = PoleLocation.None;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(0);
        clawOpenStartedTime = 0;
    }
}
