package org.firstinspires.ftc.teamcode.robovalley;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="RedFarAutoTest")
public class RedFarAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90)); //Pose2d(14, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose).
                strafeTo(new Vector2d(20, 22)).
                waitSeconds(0.5).
                strafeTo(new Vector2d(20, 46)).
                waitSeconds(0.5);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build()
                    )
            );
            requestOpModeStop();

        }
    }
}
