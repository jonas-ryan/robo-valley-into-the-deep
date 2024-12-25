package org.firstinspires.ftc.teamcode.robovalley;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Robot Centric", group="Mecanum Drive")
//@Disabled
public class MecanumDriveRobotCentricV2 extends LinearOpMode {

    private double slowSpeed = 0.333;
    private double speedMultiplier = 1.0;
    private double slideSlowSpeed = 0.333;
    private int slideVertSafetyMaximum = 4200; // 4374
    private int slideVertSafetyMinimum = 500;
    private double slideVertSafetySpeed = 0.2;
    private double slideVertBrakePower = 0.1;
    private int slidePerpSafetyMaximum = 2000; // 2173 
    private int slidePerpSafetyMinimum = 500;
    private double slidePerpSafetySpeed = 0.2;
    private enum slideHeight {
        DONE,
        FLOOR,
        WALL,
        CHAMBER
    }
    private slideHeight currentSlideHeight = slideHeight.DONE;
    private int slideHeightWall = 1150;
    private int slideHeightFloor = 350;
    private int slideHeightChamber = 2750;
    private double wristPosition = 0.05;

    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx rightFront = null;
    private Servo wrist = null;

    private DcMotorEx backSlide = null;
    private DcMotorEx frontSlide = null;
    private DcMotorEx perpSlide = null;
    private CRServo intakeA = null;
    private CRServo intakeB = null;


    @Override
    public void runOpMode() {

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
        wrist = hardwareMap.get(Servo.class, "wrist");

        backSlide = hardwareMap.get(DcMotorEx.class, "backSlide");
        frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
        perpSlide = hardwareMap.get(DcMotorEx.class, "perpSlide");
        backSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        frontSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        perpSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //backSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //perpSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        backSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        perpSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeA = hardwareMap.get(CRServo.class, "intakeA");
        intakeB = hardwareMap.get(CRServo.class, "intakeB");
        intakeA.setDirection(CRServo.Direction.REVERSE);
        intakeB.setDirection(CRServo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            /* CONTROLS */

            double controlAxial = -gamepad1.right_stick_y;
            double controlLateral = gamepad1.right_stick_x;
            double controlYaw = gamepad1.left_stick_x;
            boolean controlSlow = gamepad1.left_bumper;
            boolean controlReverse = gamepad1.right_bumper;
            boolean controlWristLower = gamepad1.a;
            boolean controlWristRaise = gamepad1.b;

            double controlVertSlide = -gamepad2.left_stick_y;
            double controlPerpSlide = -gamepad2.right_stick_y;
            double controlIntake =  gamepad2.right_trigger - gamepad2.left_trigger;
            boolean controlSlideOverride = gamepad2.dpad_up;
            boolean controlResetVertSlide = gamepad1.dpad_down && gamepad1.dpad_right || gamepad2.dpad_down && gamepad2.dpad_right;
            boolean controlResetPerpSlide = gamepad1.dpad_down && gamepad1.dpad_left || gamepad2.dpad_down && gamepad2.dpad_left;
            boolean controlSlideFloor = gamepad2.a;
            boolean controlSlideChamber = gamepad2.y;
            boolean controlSlideWall = gamepad2.b;
            boolean controlSlideDone = gamepad2.x;
            boolean controlSlideSlow = gamepad2.right_bumper;


            /* DRIVETRAIN CODE */

            if (controlSlow) {
                controlAxial *= slowSpeed;
                controlLateral *= slowSpeed;
                controlYaw *= slowSpeed;
            }
            if (controlReverse) {
                controlAxial *= -1;
                controlLateral *= -1;
            }

            double max;

            double leftFrontPower = controlAxial + controlLateral + controlYaw;
            double leftBackPower = controlAxial - controlLateral + controlYaw;
            double rightBackPower = controlAxial + controlLateral - controlYaw;
            double rightFrontPower = controlAxial - controlLateral - controlYaw;

            leftFrontPower *= speedMultiplier;
            leftBackPower *= speedMultiplier;
            rightBackPower *= speedMultiplier;
            rightFrontPower *= speedMultiplier;

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


            /*   ARM CODE   */

            if (controlWristRaise) {
                wrist.setPosition(0.3);
            }
            if (controlWristLower) {
                wrist.setPosition(0.01);
            }


            intakeA.setPower(controlIntake);
            intakeB.setPower(controlIntake);

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
                if (backSlide.getCurrentPosition() > slideVertSafetyMaximum && controlVertSlide > 0 && !controlSlideOverride) {
                    backSlide.setPower(0);
                    frontSlide.setPower(0);
                } else if (backSlide.getCurrentPosition() < slideVertSafetyMinimum && controlVertSlide < 0 && !controlSlideOverride) {
                    backSlide.setPower(controlVertSlide * slideVertSafetySpeed);
                    frontSlide.setPower(controlVertSlide * slideVertSafetySpeed);
                } else {
                    backSlide.setPower(controlVertSlide);
                    frontSlide.setPower(controlVertSlide);
                }
            } else {
                if (currentSlideHeight == slideHeight.WALL) {
                    if (Math.abs(backSlide.getCurrentPosition() - slideHeightWall) < 50) {
                        currentSlideHeight = slideHeight.DONE;
                    } else if (backSlide.getCurrentPosition() < slideHeightWall) {
                        backSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slideHeightWall) / 400.0));
                        frontSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slideHeightWall) / 400.0));
                    } else if (backSlide.getCurrentPosition() > slideHeightWall) {
                        backSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slideHeightWall) / 400.0));
                        frontSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slideHeightWall) / 400.0));
                    }
                }
                if (currentSlideHeight == slideHeight.CHAMBER) {
                    if (Math.abs(backSlide.getCurrentPosition() - slideHeightChamber) < 50) {
                        currentSlideHeight = slideHeight.DONE;
                    } else if (backSlide.getCurrentPosition() < slideHeightChamber) {
                        backSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slideHeightChamber) / 400.0));
                        frontSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slideHeightChamber) / 400.0));
                    } else if (backSlide.getCurrentPosition() > slideHeightChamber) {
                        backSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slideHeightChamber) / 400.0));
                        frontSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slideHeightChamber) / 400.0));
                    }
                }
                if (currentSlideHeight == slideHeight.FLOOR) {
                    if (Math.abs(backSlide.getCurrentPosition() - slideHeightFloor) < 50) {
                        currentSlideHeight = slideHeight.DONE;
                    } else if (backSlide.getCurrentPosition() < slideHeightFloor) {
                        backSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slideHeightFloor) / 400.0));
                        frontSlide.setPower((Math.abs(backSlide.getCurrentPosition() - slideHeightFloor) / 400.0));
                    } else if (backSlide.getCurrentPosition() > slideHeightFloor) {
                        backSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slideHeightFloor) / 400.0));
                        frontSlide.setPower(-(Math.abs(backSlide.getCurrentPosition() - slideHeightFloor) / 400.0));
                    }
                }
            }

            if (controlSlideSlow) {
                backSlide.setPower(backSlide.getPower() * slideSlowSpeed);
                frontSlide.setPower(frontSlide.getPower() * slideSlowSpeed);
            }

            if (backSlide.getCurrentPosition() > 50) {
                backSlide.setPower(backSlide.getPower() + slideVertBrakePower);
                frontSlide.setPower(frontSlide.getPower() + slideVertBrakePower);
            }

            if (perpSlide.getCurrentPosition() > slidePerpSafetyMaximum && controlPerpSlide > 0 && !controlSlideOverride) {
                perpSlide.setPower(0);
            } else if (perpSlide.getCurrentPosition() < slidePerpSafetyMinimum && controlPerpSlide < 0 && !controlSlideOverride) {
                perpSlide.setPower(controlPerpSlide * slidePerpSafetySpeed);
            } else {
                perpSlide.setPower(controlPerpSlide);
            }


            /* CONFIG BUTTONS */

            if (controlResetVertSlide) {
                backSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (controlResetPerpSlide) {
                perpSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                perpSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Telemetry updates in case there is any telemetry that needs to be displayed.
            telemetry.addLine(Float.toString(-gamepad2.left_stick_y));
            telemetry.addLine(Float.toString(gamepad2.left_stick_x));
            telemetry.addLine(Float.toString(backSlide.getCurrentPosition()));
            telemetry.addLine(Float.toString(frontSlide.getCurrentPosition()));
            telemetry.addLine(Float.toString(perpSlide.getCurrentPosition()));
            telemetry.update();
        }
    }}
