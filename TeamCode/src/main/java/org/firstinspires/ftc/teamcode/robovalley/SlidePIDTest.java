package org.firstinspires.ftc.teamcode.robovalley;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="SlidePIDTest")
public class SlidePIDTest extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    private final double ticksInInch = 114.9722222222224;

    private DcMotorEx linearSlide;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int slidePosition = linearSlide.getCurrentPosition();
            double pid = controller.calculate(slidePosition, target);
            //double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

            //double power = pid + ff;

            linearSlide.setPower(pid);

            telemetry.addData("pos", slidePosition);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}