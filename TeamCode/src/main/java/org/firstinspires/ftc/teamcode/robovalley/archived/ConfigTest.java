package org.firstinspires.ftc.teamcode.robovalley.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
public class ConfigTest {
    public static boolean testBool = false;
    public static void setTestBool(boolean setValue) {
        testBool = setValue;
    }
    public static boolean getTestBool() {
        return testBool;
    }

    @TeleOp(name="ControlsTest")
    public static class ControlsTest extends LinearOpMode {
        private DcMotorEx linearSlide;
        private DcMotorEx wormGear;
        private double wormDegreesPerTick = 0.0250347705146036;
        private double slideInchesPerTick = 0.0086977530804542;

        @Override
        public void runOpMode() {
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
                linearSlide.setPower(-gamepad2.right_stick_y);

                if ((linearSlide.getCurrentPosition() * slideInchesPerTick * Math.cos(Math.toRadians(wormGear.getCurrentPosition() * wormDegreesPerTick))) > 19.5 && -gamepad2.left_stick_y < -0.1) {
                    linearSlide.setPower(-1);
                    wormGear.setPower(-gamepad2.left_stick_y);
                }
                if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                    wormGear.setPower(-gamepad2.left_stick_y);
                } else {
                    wormGear.setPower(0);
                }

                telemetry.addData("Slide position", linearSlide.getCurrentPosition());
                telemetry.addData("horizontal extension", linearSlide.getCurrentPosition() * slideInchesPerTick * Math.cos(Math.toRadians(wormGear.getCurrentPosition() * wormDegreesPerTick)));
                telemetry.update();

            }
        }
    }
}
