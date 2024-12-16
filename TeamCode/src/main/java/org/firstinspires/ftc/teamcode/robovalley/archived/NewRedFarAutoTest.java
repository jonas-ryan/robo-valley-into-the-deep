package org.firstinspires.ftc.teamcode.robovalley.archived;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="NewRedFarAutoTest")
@Disabled
public class NewRedFarAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90)); //Pose2d(14, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);
        WormGear wormGear = new WormGear(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose).
                strafeTo(new Vector2d(-5, 5));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-5, 5, Math.toRadians(90))).
                strafeTo(new Vector2d(-5, 18)).
                waitSeconds(1);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-5, 17, Math.toRadians(90))).
                waitSeconds(0.5).
                strafeTo(new Vector2d(10, 6));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(10, 6, Math.toRadians(90))).
                waitSeconds(1).
                strafeTo(new Vector2d(20,15)).
                strafeTo(new Vector2d(20,52)).
                strafeTo(new Vector2d(30,52)).
                strafeTo(new Vector2d(30,5)).
                strafeTo(new Vector2d(30,52)).
                strafeTo(new Vector2d(37,52)).
                strafeTo(new Vector2d(37,5)).
                strafeTo(new Vector2d(37,52)).
                strafeTo(new Vector2d(46,52)).
                strafeTo(new Vector2d(46,5)).
                strafeTo(new Vector2d(35,10));

        Actions.runBlocking(claw.closeClaw());

        waitForStart();

        while (opModeIsActive()) {
            if (isStopRequested()) return;



            Actions.runBlocking(
                    new SequentialAction(
                            tab1.build(),
                            wormGear.wormUp(),
                            linearSlide.slideUp(),
                            tab2.build(),
                            linearSlide.slideDownTiny(),
                            claw.openClaw(),
                            tab3.build(),
                            linearSlide.slideDown(),
                            wormGear.wormDown(),
                            tab4.build()
                    )
            );
            requestOpModeStop();

        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.3);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class LinearSlide {
        private DcMotorEx linearSlide;

        public LinearSlide(HardwareMap hardwareMap) {
            linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlide.setDirection(DcMotorSimple.Direction.REVERSE); //2426 //1666
        }

        public class SlideUp implements Action {
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearSlide.setPower(0.8);
                    initialized = true;
                }

                double position = linearSlide.getCurrentPosition();
                packet.put("liftPosition", position);
                if (position < 2426.0) {
                    return true;
                } else {
                    linearSlide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideUp() {
            return new SlideUp();
        }

        public class SlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearSlide.setPower(-0.8);
                    initialized = true;
                }

                double position = linearSlide.getCurrentPosition();
                packet.put("liftPosition", position);
                if (position > 100.0) {
                    return true;
                } else {
                    linearSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action slideDown() {
            return new SlideDown();
        }

        public class SlideDownTiny implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    linearSlide.setPower(-0.5);
                    initialized = true;
                }

                double position = linearSlide.getCurrentPosition();
                packet.put("liftPosition", position);
                if (position > 1600.0) {
                    return true;
                } else {
                    linearSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action slideDownTiny() {
            return new SlideDownTiny();
        }




    }

    public class WormGear {
        private DcMotorEx wormGear;

        public WormGear(HardwareMap hardwareMap) {
            wormGear = hardwareMap.get(DcMotorEx.class, "wormGear");
            wormGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wormGear.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class WormUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    wormGear.setPower(1);
                    initialized = true;
                }

                double position = wormGear.getCurrentPosition();
                packet.put("wormPosition", position);
                if (position < 1666.0) {
                    return true;
                } else {
                    wormGear.setPower(0);
                    return false;
                }
            }
        }

        public Action wormUp() {
            return new WormUp();
        }

        public class WormDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    wormGear.setPower(-1);
                    initialized = true;
                }

                double position = wormGear.getCurrentPosition();
                packet.put("wormPosition", position);
                if (position > 50.0) {
                    return true;
                } else {
                    wormGear.setPower(0);
                    return false;
                }
            }
        }
        public Action wormDown() {
            return new WormDown();
        }
    }



}

