package org.firstinspires.ftc.teamcode.robovalley;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class ConfigTest {
    public static boolean testBool = false;
    public static void setTestBool(boolean setValue) {
        testBool = setValue;
    }
    public static boolean getTestBool() {
        return testBool;
    }
}
