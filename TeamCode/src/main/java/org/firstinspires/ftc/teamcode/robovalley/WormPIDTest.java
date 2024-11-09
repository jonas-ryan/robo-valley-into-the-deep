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
@TeleOp(name="WormPIDTest")
public class WormPIDTest extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.01, i = 0.05, d = 0.0002;
    public static double f = 0.01;

    public static int target = 0;

    private final double ticksInDegree = 39.94444444444447;

    private DcMotorEx wormGear;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wormGear = hardwareMap.get(DcMotorEx.class, "wormGear");
        wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            controller.setPID(p, i, d);
            int armPosition = wormGear.getCurrentPosition();
            double pid = controller.calculate(armPosition, target);
            double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

            double power = pid + ff;

            wormGear.setPower(power);

            telemetry.addData("pos", armPosition);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}