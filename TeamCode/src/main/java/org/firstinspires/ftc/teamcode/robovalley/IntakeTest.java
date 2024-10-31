package org.firstinspires.ftc.teamcode.robovalley;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="IntakeTest")
//@Disabled
public class IntakeTest extends LinearOpMode {

    private CRServo intake = null;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                intake.setPower(0.2);
            } else if (gamepad1.b) {
                intake.setPower(intake.getPower() - 0.2);
            } else {
                intake.setPower(0);
            }

        }
    }
}