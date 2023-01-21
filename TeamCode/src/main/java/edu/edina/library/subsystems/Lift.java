package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.sql.Time;

import edu.edina.library.util.ClawServoPosition;
import edu.edina.library.util.ArmServoPosition;
import edu.edina.library.util.PoleLocation;
import edu.edina.library.util.RobotState;

public class Lift extends edu.edina.library.subsystems.Subsystem {

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
    private long liftMotorStartedTime = 0;
    private boolean liftMotorReset = false;
    private boolean clawOpen = false;

    public Lift(HardwareMap map, RobotState robotState) {
        try {
            liftMotor = map.get(DcMotorEx.class, "liftMotor");
            armServo = map.get(Servo.class, "armServo");
            clawServo = map.get(Servo.class, "clawServo");
            liftSwitch = map.get(DigitalChannel.class, "liftSwitch");

            // set the digital channel to input.
            liftSwitch.setMode(DigitalChannel.Mode.INPUT);

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.5);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
            robotState.ClawServoPosition = ClawServoPosition.Closed;

            armServo.setPosition(robotState.ARMFRONTPOSITION);
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
                    clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Open;
                    clawOpenStartedTime = System.currentTimeMillis();
                    runningToPosition = true;
                    clawOpen = false;
                } else if (!clawOpen) {
                    if ((System.currentTimeMillis() > (clawOpenStartedTime + robotState.CLAWOPENWAITTIME)) && (Math.round(clawServo.getPosition() * 100) == robotState.CLAWOPENPOSITION100)) {
                        liftMotor.setTargetPosition(robotState.LIFTRETURNHEiGHT);
                        liftMotor.setPower(1);
                        clawOpen = true;
                        atZeroPosition = false;
                    }
                } else if (!atZeroPosition) {
                    robotState.LiftDiff = Math.abs(liftMotor.getCurrentPosition());

                    if (robotState.LiftDiff < 10) {
                        resetState();
                    }
                }
            } else {
                if (!runningToPosition) {
                    if (robotState.TargetPoleLocation == PoleLocation.Low) {
                        targetPosition = robotState.POLEPOSITIONLOW;
                    } else if (robotState.TargetPoleLocation == PoleLocation.Medium) {
                        targetPosition = robotState.POLEPOSITIONMIDDLE;
                    } else if (robotState.TargetPoleLocation == PoleLocation.High) {
                        targetPosition = robotState.POLEPOSITIONHIGH;
                    }

                    clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    robotState.ClawServoPosition = ClawServoPosition.Closed;
                    liftMotor.setTargetPosition(targetPosition);
                    liftMotor.setPower(1);
                    runningToPosition = true;
                } else {
                    if (robotState.ClawServoPosition == ClawServoPosition.Open) {
                        clawServo.setPosition(robotState.CLAWOPENPOSITION);
                    } else if (robotState.ClawServoPosition == ClawServoPosition.Closed) {
                        clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
                    }

                    if (robotState.ArmServoPosition == ArmServoPosition.Front) {
                        armServo.setPosition(robotState.ARMFRONTPOSITION);
                    } else if (robotState.ArmServoPosition == ArmServoPosition.Side) {
                        armServo.setPosition(robotState.ARMSIDEPOSITION);
                    } else if (robotState.ArmServoPosition == ArmServoPosition.Back) {
                        armServo.setPosition(robotState.ARMBACKPOSTITION);
                    }
                }
            }
        } else {
            liftMotor.setTargetPosition(this.targetPosition);
        }

        if (robotState.ClawServoPosition == ClawServoPosition.Open) {
            clawServo.setPosition(robotState.CLAWOPENPOSITION);
        } else if (robotState.ClawServoPosition == ClawServoPosition.Closed) {
            clawServo.setPosition(robotState.CLAWCLOSEDPOSITION);
        }

        if (robotState.ArmServoPosition == ArmServoPosition.Front) {
            armServo.setPosition(robotState.ARMFRONTPOSITION);
        } else if (robotState.ArmServoPosition == ArmServoPosition.Side) {
            armServo.setPosition(robotState.ARMSIDEPOSITION);
        } else if (robotState.ArmServoPosition == ArmServoPosition.Back) {
            armServo.setPosition(robotState.ARMBACKPOSTITION);
        }

        if (!liftSwitch.getState() && !liftMotorReset) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotorReset = true;
        } else {
            liftMotorReset = false;
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
        if ((System.currentTimeMillis() > (liftMotorStartedTime + robotState.LIFTWAITTIME))) {
            liftMotorStartedTime = System.currentTimeMillis();
            if (liftUp != 0)
                this.targetPosition += 10;
            else if (liftDown != 0)
                this.targetPosition += -10;
            else
                this.liftSpeed = 0;
            if (targetPosition >= 0){
                targetPosition = 0;
            }
        }
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
        liftMotor.setPower(0);
        clawOpenStartedTime = 0;
        liftMotorStartedTime = 0;
    }
}
