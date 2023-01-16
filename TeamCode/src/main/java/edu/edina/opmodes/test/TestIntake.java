package edu.edina.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.Stickygamepad;

@TeleOp
//@Disabled
public class TestIntake extends LinearOpMode {
    private Servo clampServo;
    private Servo armServo;

    @Override
    public void runOpMode() throws InterruptedException {
        Stickygamepad pad1 = new Stickygamepad(gamepad1);

        clampServo = hardwareMap.get(Servo.class, "clampServo");
        armServo = hardwareMap.get(Servo.class, "armServo");

        // set the digital channel to input.
        armServo.setPosition(.5);

        clampServo.setPosition(.5);

        waitForStart();

        while (opModeIsActive()) {
            pad1.update();

            if (pad1.dpad_left) {
                clampServo.setPosition(clampServo.getPosition() + .01);
            }

            if (pad1.dpad_right) {
                clampServo.setPosition(clampServo.getPosition() - .01);
            }

            if (pad1.x) {
                armServo.setPosition(armServo.getPosition() + .01);
            }

            if (pad1.b) {
                armServo.setPosition(armServo.getPosition() - .01);
            }

            telemetry.addData("Clamp Servo", clampServo.getPosition());
            telemetry.addData("Arm Servo", armServo.getPosition());
            telemetry.update();
        }
    }
}
