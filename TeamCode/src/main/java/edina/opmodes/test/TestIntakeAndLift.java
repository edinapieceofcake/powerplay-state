package edina.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.Stickygamepad;

@TeleOp
@Disabled
public class TestIntakeAndLift extends LinearOpMode {
    public static double MAXFLIPPOSITION = 1;
    public static double MINFLIPPOSITION = 0;
    public static double TRANSFERPOSITION = .25;
    public static double MIDDLEPOSITION = .45;
    public static double INCREMENTFLIP = .02;
    public static int INCREMENTTIMEOUT = 35;

    private DcMotorEx slideMotor;
    private Servo clampServo;
    private Servo armFlipServo;
    private DigitalChannel slideSwitch;

    private long lastUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        Stickygamepad pad2 = new Stickygamepad(gamepad2);
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        Servo liftFlipServo = hardwareMap.get(Servo.class, "liftFlipServo");
        Servo elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbowServo.setPosition(.6);
        clawServo.setPosition(0.55);
        liftFlipServo.setPosition(.6);

        Stickygamepad pad1 = new Stickygamepad(gamepad1);
        boolean flipUp = false;
        boolean flipDown = false;
        double FlipPosition = 0;

        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        clampServo = hardwareMap.get(Servo.class, "clampServo");
        armFlipServo = hardwareMap.get(Servo.class, "armFlipServo");
        slideSwitch = hardwareMap.get(DigitalChannel.class, "slideSwitch");

        // set the digital channel to input.
        slideSwitch.setMode(DigitalChannel.Mode.INPUT);

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armFlipServo.setPosition(.45);

        clampServo.setPosition(.6);

        waitForStart();

        while (opModeIsActive()) {
            pad2.update();

            if (pad2.dpad_left) {
                elbowServo.setPosition(elbowServo.getPosition() + 0.01);
            }
            if (pad2.dpad_right) {
                elbowServo.setPosition(elbowServo.getPosition() - 0.01);
            }

            if (pad2.right_bumper) {
                clawServo.setPosition(clawServo.getPosition() + 0.01);
            }

            if (pad2.left_bumper) {
                clawServo.setPosition(clawServo.getPosition() - 0.01);
            }

            if (pad2.dpad_up) {
                liftFlipServo.setPosition(liftFlipServo.getPosition() + 0.01);
            }
            if (pad2.dpad_down) {
                liftFlipServo.setPosition(liftFlipServo.getPosition() - 0.01);
            }

            if (gamepad2.right_trigger != 0) {
                liftMotor.setPower(1);
            } else if (gamepad2.left_trigger != 0) {
                liftMotor.setPower(-1);
            } else {
                liftMotor.setPower(0);
            }

            telemetry.addData("Motor Position", liftMotor.getCurrentPosition());
            telemetry.addData("Elbow Position", elbowServo.getPosition());
            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.addData("LiftFlip Position", liftFlipServo.getPosition());

            flipUp = gamepad1.left_bumper;
            flipDown = gamepad1.right_bumper;
            pad1.update();

            if (System.currentTimeMillis() > (lastUpdate + INCREMENTTIMEOUT) && (flipUp || flipDown)) {
                if (flipUp) {
                    FlipPosition += INCREMENTFLIP;
                } else if (flipDown) {
                    FlipPosition -= INCREMENTFLIP;
                }

                FlipPosition = Math.min(MAXFLIPPOSITION, Math.max(MINFLIPPOSITION, FlipPosition));
                armFlipServo.setPosition(FlipPosition);
                lastUpdate = System.currentTimeMillis();
            }

            if (gamepad1.left_trigger != 0) {
                slideMotor.setPower(-1);
            } else if (gamepad1.right_trigger != 0) {
                slideMotor.setPower(1);
            } else {
                slideMotor.setPower(0);
            }

            if (pad1.dpad_left) {
                clampServo.setPosition(.5);
            }

            if (pad1.dpad_right) {
                clampServo.setPosition(.75);
            }

            telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
            telemetry.addData("Clamp Servo", clampServo.getPosition());
            telemetry.addData("Arm Flip Servo", armFlipServo.getPosition());
            telemetry.update();
        }
    }
}
