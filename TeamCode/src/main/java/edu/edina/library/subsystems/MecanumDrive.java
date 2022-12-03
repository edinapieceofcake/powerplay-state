package edu.edina.library.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.Collections;

import edu.edina.library.util.RobotState;

public class MecanumDrive extends edu.edina.library.subsystems.Subsystem {

    private DcMotorEx[] motors;
    public static final String[] MOTOR_NAMES = {"leftFront", "leftRear", "rightRear", "rightFront"};
    private double[] powers;
    private double leftStickX;
    private double leftStickY;
    private double rightStickY;
    private double driveStickSpeed = .8;
    private double rotationStickSpeed = .8;

    private double currentPower = 1;

    public MecanumDrive(HardwareMap map, RobotState robotState) {
        powers = new double[4];
        motors = new DcMotorEx[4];

        try {
            for (int i = 0; i < 4; i ++) {
                DcMotorEx dcMotor = map.get(DcMotorEx.class, MOTOR_NAMES[i]);
                motors[i] = dcMotor;
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
            motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
            robotState.DriveSuccessfullySetup = true;
        } catch (Exception ex) {
            robotState.DriveSuccessfullySetup = false;
        }
    }

    public void setDriveProperties(double leftStickX, double leftStickY, double rightStickY,
                                   boolean lowSpeed, boolean mediumSpeed, boolean highSpeed) {
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickY = rightStickY;

        if (lowSpeed) {
            currentPower = .5;
        } else if (mediumSpeed) {
            currentPower = 1.0;
        } else if (highSpeed) {
            currentPower = 1.4;
        }
    }

    public void update() {
        double x;
        double y;
        double rotation;
        double speed;

        x = Math.pow(-leftStickX, 3.0);
        y = Math.pow(leftStickY, 3.0);
        rotation = Math.pow(-rightStickY, 3.0) * rotationStickSpeed;
        speed = Math.min(1.0, Math.sqrt(x * x + y * y)) * driveStickSpeed;

        final double direction = Math.atan2(x, y);

        powers[0] = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * currentPower;
        powers[3] = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * currentPower;
        powers[1] = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * currentPower;
        powers[2] = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * currentPower;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(powers[0]),
                Math.abs(powers[1]), Math.abs(powers[2]), Math.abs(powers[3])));

        for (int i = 0; i < 4; i++) {
            powers[i] /= max;
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }
}