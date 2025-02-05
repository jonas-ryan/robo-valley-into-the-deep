package org.firstinspires.ftc.teamcode.robovalley;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@TeleOp(name="Robot Centric v2", group="Mecanum Drive")
//@Disabled
public class MecanumDriveRobotCentricV3 extends LinearOpMode {

    /* DRIVETRAIN VARIABLES */
    private final double DRIVETRAIN_SLOW_SPEED = 0.333;
    private final double DRIVETRAIN_SPEED_MULTIPLIER = 1.0;

    /* SLIDE VARIABLES */
    private final double SLIDES_SLOW_SPEED = 0.333;
    private final int VERTICAL_SLIDE_SAFETY_MAXIMUM = 4200; // 4374
    private final int VERTICAL_SLIDE_SAFETY_MINIMUM = 500;
    private final double VERTICAL_SLIDE_SAFETY_SPEED = 0.2;
    private final double VERTICAL_SLIDE_BRAKE_POWER = 0.1;
    private final int PERPENDICULAR_SLIDE_MAXIMUM = 2000; // 2173
    private final int PERPENDICULAR_SLIDE_MINIMUM = 500;
    private final double PERPENDICULAR_SLIDE_MINIMUM_SPEED = 0.2;
    private enum slideHeight {
        DONE,
        FLOOR,
        WALL,
        CHAMBER
    }
    private slideHeight currentSlideHeight = slideHeight.DONE;
    private final int SLIDES_HEIGHT_WALL = 1100;
    private final int SLIDES_HEIGHT_FLOOR = 350;
    private final int SLIDES_HEIGHT_CHAMBER = 2850;
    private HashMap<slideHeight, Integer> slidesHeights = new HashMap<>();

    /* DRIVETRAIN MOTORS */
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx rightFront = null;

    /* SLIDES MOTORS */
    private DcMotorEx backSlide = null;
    private DcMotorEx frontSlide = null;
    private DcMotorEx perpendicularSlide = null;

    /* INTAKE SERVOS */
    private Servo wrist = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;


    @Override
    public void runOpMode() {
        /* DRIVETRAIN CONFIGURATION */
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /* SLIDES CONFIGURATION */
        backSlide = hardwareMap.get(DcMotorEx.class, "backSlide");
        frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
        perpendicularSlide = hardwareMap.get(DcMotorEx.class, "perpSlide");
        backSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        frontSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        perpendicularSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        backSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        perpendicularSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* INTAKE CONFIGURATION */
        leftIntake = hardwareMap.get(CRServo.class, "intakeA");
        rightIntake = hardwareMap.get(CRServo.class, "intakeB");
        leftIntake.setDirection(CRServo.Direction.REVERSE);
        rightIntake.setDirection(CRServo.Direction.FORWARD);
        wrist = hardwareMap.get(Servo.class, "wrist");

        /* HASHMAP CONFIGURATION */
        slidesHeights.put(slideHeight.WALL, SLIDES_HEIGHT_WALL);
        slidesHeights.put(slideHeight.FLOOR, SLIDES_HEIGHT_FLOOR);
        slidesHeights.put(slideHeight.CHAMBER, SLIDES_HEIGHT_CHAMBER);

        /* TELEMETRY */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            /* DRIVETRAIN CONTROLS */
            double controlAxial = -gamepad1.right_stick_y;
            double controlLateral = gamepad1.right_stick_x;
            double controlYaw = gamepad1.left_stick_x;
            boolean controlSlow = gamepad1.left_bumper;
            boolean controlReverse = gamepad1.right_bumper;

            /* SLIDES CONTROLS */
            double controlVerticalSlide = -gamepad2.left_stick_y;
            double controlPerpendicularSlide = -gamepad2.right_stick_y;
            boolean controlSlideOverride = gamepad2.dpad_up;
            boolean controlResetVerticalSlide = gamepad1.dpad_down && gamepad1.dpad_right || gamepad2.dpad_down && gamepad2.dpad_right;
            boolean controlResetPerpendicularSlide = gamepad1.dpad_down && gamepad1.dpad_left || gamepad2.dpad_down && gamepad2.dpad_left;
            boolean controlSlideFloor = gamepad2.a;
            boolean controlSlideChamber = gamepad2.y;
            boolean controlSlideWall = gamepad2.b;
            boolean controlSlideDone = gamepad2.x;
            boolean controlSlideSlow = gamepad2.right_bumper;

            /* INTAKE CONTROLS */
            double controlIntake =  gamepad2.right_trigger - gamepad2.left_trigger;
            boolean controlWristRaise = gamepad1.y;
            boolean controlWristFlat = gamepad1.x;
            boolean controlWristLower = gamepad1.a;

            ////////////////////

            /* DRIVETRAIN CODE */
            if (controlSlow) {
                controlAxial *= DRIVETRAIN_SLOW_SPEED;
                controlLateral *= DRIVETRAIN_SLOW_SPEED;
                controlYaw *= DRIVETRAIN_SLOW_SPEED;
            }
            if (controlReverse) {
                controlAxial *= -1;
                controlLateral *= -1;
            }
            controlAxial *= DRIVETRAIN_SPEED_MULTIPLIER;
            controlLateral *= DRIVETRAIN_SPEED_MULTIPLIER;
            controlYaw *= DRIVETRAIN_SPEED_MULTIPLIER;

            double leftFrontPower = controlAxial + controlLateral + controlYaw;
            double leftBackPower = controlAxial - controlLateral + controlYaw;
            double rightBackPower = controlAxial + controlLateral - controlYaw;
            double rightFrontPower = controlAxial - controlLateral - controlYaw;

            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            max = Math.max(max, Math.abs(rightFrontPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            /* SLIDES CODE */
            if (controlSlideFloor) {
                currentSlideHeight = slideHeight.FLOOR;
            }
            if (controlSlideDone) {
                currentSlideHeight = slideHeight.DONE;
            }
            if (controlSlideChamber) {
                currentSlideHeight = slideHeight.CHAMBER;
            }
            if (controlSlideWall) {
                currentSlideHeight = slideHeight.WALL;
            }
            if (currentSlideHeight == slideHeight.DONE) {
                if (backSlide.getCurrentPosition() > VERTICAL_SLIDE_SAFETY_MAXIMUM && controlVerticalSlide > 0 && !controlSlideOverride) {
                    backSlide.setPower(0);
                    frontSlide.setPower(0);
                } else if (backSlide.getCurrentPosition() < 0 && controlVerticalSlide < 0 && !controlSlideOverride) {
                    backSlide.setPower(0);
                    frontSlide.setPower(0);
                } else if (backSlide.getCurrentPosition() < VERTICAL_SLIDE_SAFETY_MINIMUM && controlVerticalSlide < 0 && !controlSlideOverride) {
                    backSlide.setPower(controlVerticalSlide * VERTICAL_SLIDE_SAFETY_SPEED);
                    frontSlide.setPower(controlVerticalSlide * VERTICAL_SLIDE_SAFETY_SPEED);
                } else {
                    backSlide.setPower(controlVerticalSlide);
                    frontSlide.setPower(controlVerticalSlide);
                }
            } else {
                if (slidesHeights.get(currentSlideHeight) != null) {
                    if (Math.abs(backSlide.getCurrentPosition() - slidesHeights.get(currentSlideHeight)) < 50) {
                        currentSlideHeight = slideHeight.DONE;
                    } else if (backSlide.getCurrentPosition() < slidesHeights.get(currentSlideHeight)) {
                        backSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slidesHeights.get(currentSlideHeight)) / 400.0));
                        frontSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slidesHeights.get(currentSlideHeight)) / 400.0));
                    } else if (backSlide.getCurrentPosition() > slidesHeights.get(currentSlideHeight)) {
                        backSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slidesHeights.get(currentSlideHeight)) / 400.0));
                        frontSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slidesHeights.get(currentSlideHeight)) / 400.0));
                    }
                }
            }

            if (controlSlideSlow) {
                backSlide.setPower(backSlide.getPower() * SLIDES_SLOW_SPEED);
                frontSlide.setPower(frontSlide.getPower() * SLIDES_SLOW_SPEED);
            }

            if (backSlide.getCurrentPosition() > 50) {
                backSlide.setPower(backSlide.getPower() + VERTICAL_SLIDE_BRAKE_POWER);
                frontSlide.setPower(frontSlide.getPower() + VERTICAL_SLIDE_BRAKE_POWER);
            }

            if (perpendicularSlide.getCurrentPosition() > PERPENDICULAR_SLIDE_MAXIMUM && controlPerpendicularSlide > 0 && !controlSlideOverride) {
                perpendicularSlide.setPower(0);
            } else if (perpendicularSlide.getCurrentPosition() < PERPENDICULAR_SLIDE_MINIMUM && controlPerpendicularSlide < 0 && !controlSlideOverride) {
                perpendicularSlide.setPower(controlPerpendicularSlide * PERPENDICULAR_SLIDE_MINIMUM_SPEED);
            } else {
                perpendicularSlide.setPower(controlPerpendicularSlide);
            }

            /* INTAKE CODE */
            if (controlWristRaise) {
                wrist.setPosition(0.3);
            } else if (controlWristFlat) {
                wrist.setPosition(0.6);
            } else if (controlWristLower) {
                wrist.setPosition(0.95);
            }

            wrist.setPosition(wrist.getPosition() + (gamepad1.right_trigger - gamepad1.left_trigger) * 0.015);

            leftIntake.setPower(controlIntake);
            rightIntake.setPower(controlIntake);

            /* CONFIG BUTTONS */
            if (controlResetVerticalSlide) {
                backSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (controlResetPerpendicularSlide) {
                perpendicularSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                perpendicularSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Telemetry updates in case there is any telemetry that needs to be displayed.
            telemetry.addData("Back Slide", Float.toString(backSlide.getCurrentPosition()));
            telemetry.addData("Front Slide", Float.toString(frontSlide.getCurrentPosition()));
            telemetry.addData("Perpendicular Slide", Float.toString(perpendicularSlide.getCurrentPosition()));
            telemetry.addData("Runtime", getRuntime());
            telemetry.addData("Servo Position", wrist.getPosition());
            telemetry.update();
        }
    }}
