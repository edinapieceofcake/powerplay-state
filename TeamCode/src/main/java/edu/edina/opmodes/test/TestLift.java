package edu.edina.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import edu.edina.library.util.Stickygamepad;

@TeleOp()
@Disabled
public class TestLift extends LinearOpMode {
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

        waitForStart();

        while(opModeIsActive()){
            pad2.update();

            if (pad2.dpad_left){
                elbowServo.setPosition(elbowServo.getPosition()+0.05);
            }
            if (pad2.dpad_right){
                elbowServo.setPosition(elbowServo.getPosition()-0.05);
            }

            if (pad2.right_bumper){
                clawServo.setPosition(clawServo.getPosition()+0.05);
            }

            if (pad2.left_bumper){
                clawServo.setPosition(clawServo.getPosition()-0.05);
            }

            if (pad2.dpad_up){
                liftFlipServo.setPosition(liftFlipServo.getPosition()+0.05);
            }
            if (pad2.dpad_down){
                liftFlipServo.setPosition(liftFlipServo.getPosition()-0.05);
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
            telemetry.addData("Flip Position", liftFlipServo.getPosition());
            telemetry.update();
        }
    }
}
