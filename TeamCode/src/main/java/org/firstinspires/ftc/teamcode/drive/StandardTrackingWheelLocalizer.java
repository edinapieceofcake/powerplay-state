package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.37795276 / 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //9.42, 8.49
    public static double LATERAL_DISTANCE = 9.79; // in; distance between the left and right wheels
//    public static double LATERAL_DISTANCE = 9.75; // in; distance between the left and right wheels
//    public static double LATERAL_DISTANCE = 9.43; // in; distance between the left and right wheels
    //public static double LATERAL_DISTANCE = 9.3125; // in; distance between the left and right wheels
    //public static double LATERAL_DISTANCE = 10.4; // in; distance between the left and right wheels
    //public static double FORWARD_OFFSET = 2.17; // in; offset of the lateral wheel
    //public static double FORWARD_OFFSET = 2.56; // in; offset of the lateral wheel
    //public static double FORWARD_OFFSET = 2.17; // in; offset of the lateral wheel
    //public static double FORWARD_OFFSET = 5.0; // in; offset of the lateral wheel
    public static double FORWARD_OFFSET = 3.779; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;
    //public static double X_MULTIPLIER = 1.021; // Multiplier in the X direction
    //public static double Y_MULTIPLIER = 1.035; // Multiplier in the Y direction
    public static double X_MULTIPLIER = 1.020; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.020; // Multiplier in the Y direction

    private int previousLeft = 0;
    private int previousRight = 0;
    private int previousFront = 0;
    // .866
    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-.866, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(-.866, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, .5, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "centerEncoder"));

        hardwareMap.get(DcMotorEx.class, "leftEncoder").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "rightEncoder").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotorEx.class, "centerEncoder").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        if ((previousLeft != leftEncoder.getCurrentPosition()) ||
            (previousFront != frontEncoder.getCurrentPosition()) ||
            (previousRight != rightEncoder.getCurrentPosition())
        ) {
            Log.d("getWheelPositions", String.format("%d, %d, %d", leftEncoder.getCurrentPosition(),
                    rightEncoder.getCurrentPosition(),
                    frontEncoder.getCurrentPosition()));

            previousRight = rightEncoder.getCurrentPosition();
            previousFront = frontEncoder.getCurrentPosition();
            previousLeft = leftEncoder.getCurrentPosition();
        }
//        RobotLog.dd("getWheelPositions, ", "left, %d", leftEncoder.getCurrentPosition());
//        RobotLog.dd("getWheelPositions, ", "right, %d", rightEncoder.getCurrentPosition());
//        RobotLog.dd("getWheelPositions, ", "front, %d", frontEncoder.getCurrentPosition());

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
