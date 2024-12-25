package org.firstinspires.ftc.teamcode.robovalley;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "Obs")
public class NewNewRedObsAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d intialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
        VerticalSlides verticalSlides = new VerticalSlides(hardwareMap);
        PerpendicularSlide perpendicularSlide = new PerpendicularSlide(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        AutoSetter.Persist persist = new AutoSetter.Persist();

        waitForStart();

        while (opModeIsActive()) {

            // WAIT
            Actions.runBlocking(drive.actionBuilder(intialPose)
                    .waitSeconds(persist.getTime())
                    .build());

            // FIRST SCORE WITH PRELOAD
            Actions.runBlocking(drive.actionBuilder(intialPose)
                    .afterTime(0.0, verticalSlides.verticalSlidesUpChamber())
                    .afterTime(2.0, verticalSlides.verticalSlidesDownScore())
                    .afterTime(2.2, intake.intakeOuttake())
                    .afterTime(3, intake.intakeStop())
                    .strafeToLinearHeading(new Vector2d(-13,30.5), Math.toRadians(90))
                    .build());

            // PUSH SAMPLES IN AND TURN AROUND
            Actions.runBlocking(drive.actionBuilder(new Pose2d(-13, 30.5, Math.toRadians(90)))
                    .afterTime(0.0, verticalSlides.verticalSlidesDownWall())
                    .strafeToLinearHeading(new Vector2d(22,26), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(22,50), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(30,50), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(30,8), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(30,50), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(42,50), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(42,8), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(36,20), Math.toRadians(270))
                    .build());

            // INTAKE FROM WALL
            Actions.runBlocking(drive.actionBuilder(new Pose2d(36, 20, Math.toRadians(270)))
                    .afterTime(0.4, perpendicularSlide.perpendicularSlideOut())
                    .afterTime(0.4, verticalSlides.verticalSlidesUpWall())
                    .afterTime(0.2, intake.intakeIntake())
                    .afterTime(2.0, verticalSlides.verticalSlidesUpChamber2())
                    .afterTime(2.0, intake.intakeStop())
                    .strafeToLinearHeading(new Vector2d(36,10), Math.toRadians(270))
                    .build());

            // SCORE SPECIMEN
            Actions.runBlocking(drive.actionBuilder(new Pose2d(36, 7, Math.toRadians(270)))
                    .afterTime(0.0, perpendicularSlide.perpendicularSlideIn())
                    .afterTime(2.0, verticalSlides.verticalSlidesUpChamber2())
                    .afterTime(4.0, verticalSlides.verticalSlidesDownScore())
                    .afterTime(4.4, intake.intakeOuttake())
                    .afterTime(5.2, intake.intakeStop())
                    .strafeToLinearHeading(new Vector2d(-12,25.5), Math.toRadians(90))
                    .build());

            Actions.runBlocking(drive.actionBuilder(new Pose2d(-12, 25.5, Math.toRadians(90)))
                    .afterTime(0.0, perpendicularSlide.perpendicularSlideIn())
                    .afterTime(0.0, verticalSlides.verticalSlidesDownWall())
                    .strafeToLinearHeading(new Vector2d(35,6), Math.toRadians(270))
                    .build());


            requestOpModeStop();
        }

    }

    public class VerticalSlides {
        private DcMotorEx backSlide;
        private DcMotorEx frontSlide;

        public VerticalSlides(HardwareMap hardwareMap) {
            backSlide = hardwareMap.get(DcMotorEx.class, "backSlide");
            backSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
            frontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        } // WALL: 1075 CHAMBER: 2875

        public class VerticalSlidesUpChamber implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    backSlide.setPower(0.8);
                    frontSlide.setPower(0.8);
                    initialized = true;
                }

                double position = backSlide.getCurrentPosition();
                packet.put("verticalSlidesPosition", position);
                if (position < 2875) {
                    return true;
                } else {
                    backSlide.setPower(0.2);
                    frontSlide.setPower(0.2);
                    return false;
                }
            }
        }

        public Action verticalSlidesUpChamber() {
            return new VerticalSlidesUpChamber();
        }

        public class VerticalSlidesUpChamber2 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    backSlide.setPower(0.8);
                    frontSlide.setPower(0.8);
                    initialized = true;
                }

                double position = backSlide.getCurrentPosition();
                packet.put("verticalSlidesPosition", position);
                if (position < 2500) {
                    return true;
                } else {
                    backSlide.setPower(0.2);
                    frontSlide.setPower(0.2);
                    return false;
                }
            }
        }

        public Action verticalSlidesUpChamber2() {
            return new VerticalSlidesUpChamber2();
        }

        public class VerticalSlidesDownScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    backSlide.setPower(-0.3);
                    frontSlide.setPower(-0.3);
                    initialized = true;
                }

                double position = backSlide.getCurrentPosition();
                packet.put("verticalSlidesPosition", position);
                if (position > 2300) {
                    return true;
                } else {
                    backSlide.setPower(0);
                    frontSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action verticalSlidesDownScore() {
            return new VerticalSlidesDownScore();
        }

        public class VerticalSlidesUpWall implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    backSlide.setPower(0.8);
                    frontSlide.setPower(0.8);
                    initialized = true;
                }

                double position = backSlide.getCurrentPosition();
                packet.put("verticalSlidesPosition", position);
                if (position < 1075) {
                    return true;
                } else {
                    backSlide.setPower(0);
                    frontSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action verticalSlidesUpWall() {
            return new VerticalSlidesUpWall();
        }

        public class VerticalSlidesDownWall implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    backSlide.setPower(-0.8);
                    frontSlide.setPower(-0.8);
                    initialized = true;
                }

                double position = backSlide.getCurrentPosition();
                packet.put("verticalSlidesPosition", position);
                if (position > 1075) {
                    return true;
                } else {
                    backSlide.setPower(0);
                    frontSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action verticalSlidesDownWall() {
            return new VerticalSlidesDownWall();
        }
    }

    public class PerpendicularSlide {
        private DcMotorEx perpSlide;

        public PerpendicularSlide(HardwareMap hardwareMap) {
            perpSlide = hardwareMap.get(DcMotorEx.class, "perpSlide");
            perpSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            perpSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            perpSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class PerpendicularSlideIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    perpSlide.setPower(-0.8);
                    initialized = true;
                }

                double position = perpSlide.getCurrentPosition();
                packet.put("perpendicularSlidePosition", position);
                if (position > 200) {
                    return true;
                } else {
                    perpSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action perpendicularSlideIn() {
            return new PerpendicularSlideIn();
        }

        public class PerpendicularSlideOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    perpSlide.setPower(0.6);
                    initialized = true;
                }

                double position = perpSlide.getCurrentPosition();
                packet.put("perpendicularSlidePosition", position);
                if (position < 1300) {
                    return true;
                } else {
                    perpSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action perpendicularSlideOut() {
            return new PerpendicularSlideOut();
        }

    }

    public class Intake {
        private CRServo intakeA;
        private CRServo intakeB;

        public Intake(HardwareMap hardwareMap) {
            intakeA = hardwareMap.get(CRServo.class, "intakeA");
            intakeB = hardwareMap.get(CRServo.class, "intakeB");
            intakeA.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeB.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        /* public class IntakeIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeA.setPower(0.2);
                    intakeB.setPower(0.2);
                    initialized = true;
                }

                packet.put("intakePower", 0.2);
                return false;
            }
        } */

        public class IntakeIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeA.setPower(0.2);
                intakeB.setPower(0.2);
                packet.put("intakePower", 0.2);
                return false;
            }
        }

        public Action intakeIntake() {
            return new IntakeIntake();
        }

        public class IntakeOuttake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeA.setPower(-0.2);
                intakeB.setPower(-0.2);
                packet.put("intakePower", -0.2);
                return false;
            }
        }

        public Action intakeOuttake() {
            return new IntakeOuttake();
        }

        public class IntakeStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeA.setPower(0);
                intakeB.setPower(0);
                packet.put("intakePower", 0);
                return false;
            }
        }

        public Action intakeStop() {
            return new IntakeStop();
        }
    }

    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }
    }



}