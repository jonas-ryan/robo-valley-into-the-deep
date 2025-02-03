package org.firstinspires.ftc.teamcode.robovalley;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="V4 Bar Test")
public class V4BarTest extends LinearOpMode {

    CRServo leftIntake;
    CRServo rightIntake;
    Servo v4Bar;

    @Override
    public void runOpMode() {

        leftIntake = hardwareMap.get(CRServo.class, "intakeA");
        rightIntake = hardwareMap.get(CRServo.class, "intakeB");
        v4Bar = hardwareMap.get(Servo.class, "wrist");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                v4Bar.setPosition(0.8);
            }
            if (gamepad1.b) {
                v4Bar.setPosition(1);
            }
            if (gamepad1.y) {
                v4Bar.setPosition(0);
            }
            if (gamepad1.x) {
                v4Bar.setPosition(0.5);
            }

        }
    }
}