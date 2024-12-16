package org.firstinspires.ftc.teamcode.robovalley;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Robot Centric", group="Mecanum Drive")
@Disabled
public class MecanumDriveRobotCentric extends LinearOpMode {

    private double slowSpeed = 0.333;
    private double speedMultiplier = 0.85;
    private double slideSlowSpeed = 0.333;
    private double wormSlowSpeed = 0.333;
    private int slideSafetyMaximum = 4200;
    private int slideSafetyMinimum = 500;
    private double slideSafetySpeed = 0.2;
    private double wormDegreesPerTick = 0.0250347705146036;
    private double slideInchesPerTick = 0.0086977530804542;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightFront = null;

    private DcMotor wormGear = null;
    private DcMotor linearSlide = null;
    private CRServo clawWrist = null;
    private Servo claw = null;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wormGear = hardwareMap.get(DcMotor.class, "wormGear");
        wormGear.setDirection(DcMotorSimple.Direction.FORWARD);
        wormGear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawWrist = hardwareMap.get(CRServo.class, "clawWrist");
        clawWrist.setDirection(DcMotorSimple.Direction.FORWARD);
        claw = hardwareMap.get(Servo.class, "claw");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            /* CONTROLS */

            double controlAxial = -gamepad1.right_stick_y;
            double controlLateral =  gamepad1.right_stick_x;
            double controlYaw =  gamepad1.left_stick_x;
            boolean controlSlow =  gamepad1.left_bumper;
            boolean controlReverse =  gamepad1.right_bumper;

            double controlSlide =  -gamepad2.right_stick_y;
            double controlWorm = -gamepad2.left_stick_y;
            boolean controlSlideSlow =  gamepad2.left_bumper;
            double controlClawRotation =  gamepad2.right_trigger - gamepad2.left_trigger;
            boolean controlClawOpen = gamepad2.a;
            boolean controlClawClosed = gamepad2.b;
            boolean controlSlideOverride = gamepad2.right_bumper;

            boolean controlResetSlide = (gamepad1.dpad_down && gamepad1.dpad_right) || (gamepad2.dpad_down && gamepad2.dpad_right);
            boolean controlResetWormGear = (gamepad1.dpad_down && gamepad1.dpad_left) || (gamepad2.dpad_down && gamepad2.dpad_left);


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

            double leftFrontPower  = controlAxial + controlLateral + controlYaw;
            double leftBackPower   = controlAxial - controlLateral + controlYaw;
            double rightBackPower  = controlAxial + controlLateral - controlYaw;
            double rightFrontPower = controlAxial - controlLateral - controlYaw;

            leftFrontPower *= speedMultiplier;
            leftBackPower *= speedMultiplier;
            rightBackPower *= speedMultiplier;
            rightFrontPower *= speedMultiplier;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            max = Math.max(max, Math.abs(rightFrontPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);


            /*   ARM CODE   */

            // Reduces slide and worm gear speeds if the slow button is held.
            if (controlSlideSlow) {
                controlSlide *= slideSlowSpeed;
                controlWorm *= wormSlowSpeed;
            }
            // Checks if the linear slide is past the maximum extension and then if power is trying to be applied in that direction, set it to zero. There is also an override button that it checks for.
            if (linearSlide.getCurrentPosition() > slideSafetyMaximum && controlSlide > 0 && !controlSlideOverride) {
                controlSlide = 0;
            }
            // Checks if the linear slide is below a certain threshold and then limits the speed of the motor if it is being powered towards the starting position. There is also an override button that it checks for.
            if (linearSlide.getCurrentPosition() < slideSafetyMinimum && controlSlide < 0 && !controlSlideOverride) {
                controlSlide *= slideSafetySpeed;
            }

            // Checks if the current horizontal extension is greater than 20 (using some trigonometry) and then retract the slide.
            if ((linearSlide.getCurrentPosition() * slideInchesPerTick * Math.cos(Math.toRadians(wormGear.getCurrentPosition() * wormDegreesPerTick))) > 19.5 && !controlSlideOverride) {
                controlSlide = -1;
            }

            if (wormGear.getCurrentPosition() * wormDegreesPerTick > 90) {
                if (controlWorm > 0) {
                    controlWorm = 0;
                }
            }

            // Sets the powers of the slide and worm gear.
            linearSlide.setPower(controlSlide);
            wormGear.setPower(controlWorm);

            // Rotates the wrist based on the triggers.
            clawWrist.setPower(controlClawRotation * 0.1);

            // Checks if the claw buttons are pressed and sets the position of the claw accordingly.
            if (controlClawOpen) {
                claw.setPosition(0.3);
            } else if (controlClawClosed) {
                claw.setPosition(0);
            }


            /* CONFIG BUTTONS */

            // Resets the encoder positions of the slide or worm gear based on their according buttons.
            if(controlResetWormGear) {
                wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wormGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(controlResetSlide) {
                linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Telemetry updates in case there is any telemetry that needs to be displayed.
            telemetry.addData("Worm Gear Ticks", wormGear.getCurrentPosition());
            telemetry.addData("Linear Slide Ticks", linearSlide.getCurrentPosition());
            telemetry.addData("Worm Gear Degrees", wormGear.getCurrentPosition() * wormDegreesPerTick);
            telemetry.addData("Linear Slide Inches", linearSlide.getCurrentPosition() * slideInchesPerTick);
            telemetry.addData("Horizontal Slide Extension", (linearSlide.getCurrentPosition() * slideInchesPerTick * Math.cos(Math.toRadians(wormGear.getCurrentPosition() * wormDegreesPerTick))));
            telemetry.update();
        }
    }}
