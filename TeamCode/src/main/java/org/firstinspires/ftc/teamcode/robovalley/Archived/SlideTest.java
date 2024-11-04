package org.firstinspires.ftc.teamcode.robovalley.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="SlideTest")
@Disabled
public class SlideTest extends LinearOpMode {

    private DcMotor linearSlide = null;

    @Override
    public void runOpMode() {

        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Slide", linearSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}