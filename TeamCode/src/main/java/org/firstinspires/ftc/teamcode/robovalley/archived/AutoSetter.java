package org.firstinspires.ftc.teamcode.robovalley.archived;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="AutoSetter")
public class AutoSetter extends LinearOpMode {

    private Persist persist = new Persist();

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                persist.setTime(persist.getTime() + 0.5);
            }

            if (gamepad1.b) {
                persist.setTime(persist.getTime() - 0.5);
            }

            telemetry.addData("Time", persist.getTime());
            telemetry.update();
        }
    }

    public static class Persist {
        private static double time = 0;

        public double getTime() {
            return time;
        }
        public void setTime(Double timeSet) {
            time = timeSet;
        }
    }
}