package org.firstinspires.ftc.teamcode.robovalley.archived;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name="CoordinateTest")
@Disabled
public class CoordinateTest extends LinearOpMode {
    private final double wormDegreesPerTick = 0.0250347705146036;
    private final double slideInchesPerTick = 0.0086977530804542;

    private PIDController slideController;
    private PIDController wormController;

    private DcMotorEx linearSlide;
    private DcMotorEx wormGear;

    private int wormTarget;
    private int slideTarget;

    public static double targetX = 14;
    public static double targetY = 0;

    @Override
    public void runOpMode() {
        targetX = 14;
        targetY = 0;
        slideController = new PIDController(0.05, 0, 0.001);
        wormController = new PIDController(0.01, 0.2, 0.0002);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wormGear = hardwareMap.get(DcMotorEx.class, "wormGear");
        wormGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            int slidePosition = linearSlide.getCurrentPosition();
            int wormPosition = wormGear.getCurrentPosition();

            targetX = (targetX * Math.cos(Math.toRadians(-gamepad2.left_stick_y * 0.4))) - (targetY * Math.sin(Math.toRadians(-gamepad2.left_stick_y * 0.4)));
            targetY = (targetY * Math.cos(Math.toRadians(-gamepad2.left_stick_y * 0.4))) + (targetX * Math.sin(Math.toRadians(-gamepad2.left_stick_y * 0.4)));

            targetX += (Math.cos(Math.toRadians(wormPosition * wormDegreesPerTick)) / 7) * -gamepad2.right_stick_y;
            targetY += (Math.sin(Math.toRadians(wormPosition * wormDegreesPerTick)) / 7) * -gamepad2.right_stick_y;

            if (Math.hypot(targetX, targetY) < 14) {
                targetX = Math.cos(Math.toRadians(wormPosition * wormDegreesPerTick)) * 14.1;
                targetY = Math.sin(Math.toRadians(wormPosition * wormDegreesPerTick)) * 14.1;
            }

            if (targetX > 30) {
                targetX = 30;
            }

            if (targetY < -4) {
                targetY = -4;
            }

            if (targetX < 0) {
                targetX = 0;
            }

            slideController.setPID(0.05, 0, 0.001);
            wormController.setPID(0.01, 0.2, 0.0002);

            // Uses inverse tangent to find the angle and then divides by degrees per tick to find correct target
            // Also rounds the output and casts to an integer
            wormTarget = (int) Math.round(Math.toDegrees(Math.atan2(targetY, targetX)) / wormDegreesPerTick);
            // Uses the pythagorean theorem to find the length of slide and then divides by inches per tick to find correct target
            // Also rounds the output and casts to an integer
            slideTarget = (int) Math.round((Math.hypot(targetX, targetY) - 14) / slideInchesPerTick);

            double slidePower = pController(slidePosition, slideTarget); //slideController.calculate
            double wormPower = wormController.calculate(wormPosition, wormTarget);

            if (Math.abs(slideTarget - slidePosition) > 1) {
                linearSlide.setPower(Math.min(Math.max(slidePower, -0.5), 0.5));
            } else {
                linearSlide.setPower(0);
            }
            wormGear.setPower(wormPower);

            telemetry.addData("pos", slidePosition);
            telemetry.addData("target", targetX + ", " + targetY);
            telemetry.addData("slideTarget", slideTarget);
            telemetry.addData("angle", wormTarget);
            telemetry.update();
        }
    }
    public double pController(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        if (Math.abs(error) > 80) {
            if (error > 0) {
                return 0.1;
            } else {
                return -0.1;
            }
        }
        return 0;
    }
}

