package org.firstinspires.ftc.teamcode.robovalley.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robovalley.ConfigTest;

@TeleOp(name="", group="")
@Disabled
public class PIDTest extends LinearOpMode {

    private DcMotor motor = null;
    private double motorPosition = 0;
    private ElapsedTime runtime = new ElapsedTime();

    private double kP = 0.005;
    private double kI = 0;
    private double kD = 0.0001;

    private double previousError = 0;
    private double previousTime = 0;

    private ConfigTest localConfig = new ConfigTest();

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "leftBack");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                motorPosition = 2000;
            }
            if (gamepad1.b) {
                motorPosition = 0;
            }
            if (gamepad1.y) {
                motorPosition = 10000;
            }

            double motorDifference = gamepad1.right_stick_x;
            motorPosition += motorDifference * 5;

            double error = motorPosition - motor.getCurrentPosition();

            double P = kP * error;

            double I = 0;

            double D = kD * (error - previousError) / (runtime.time() - previousTime);
            previousError = error;
            previousTime = runtime.time();

            double PID = P + I + D;

            motor.setPower(PID);

            telemetry.addLine("P: " + P);
            telemetry.addLine("I: " + I);
            telemetry.addLine("D: " + D);
            telemetry.addLine("PID: " + PID);


            telemetry.addLine("Current Position: " + motor.getCurrentPosition());
            telemetry.addLine("Target Position: " + motorPosition);
            telemetry.addLine("Current Error: " + error);
            telemetry.update();
        }
    }
}