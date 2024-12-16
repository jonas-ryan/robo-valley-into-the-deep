package org.firstinspires.ftc.teamcode.robovalley;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Robot Centric", group="Mecanum Drive")
//@Disabled
public class MecanumDriveRobotCentricV2 extends LinearOpMode {

    private double slowSpeed = 0.333;
    private double speedMultiplier = 0.85;
    private double slideSlowSpeed = 0.333;
    private double wormSlowSpeed = 0.333;
    private int slideSafetyMaximum = 4200;
    private int slideSafetyMinimum = 500;
    private double slideSafetySpeed = 0.2;
    private double wormDegreesPerTick = 0.0250347705146036;
    private double slideInchesPerTick = 0.0086977530804542;

    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx rightFront = null;

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

        backSlide = hardwareMap.get(DcMotorEx.class, "backSlide");
        frontSlide = hardwareMap.get(DcMotorEx.class, "frontSlide");
        perpSlide = hardwareMap.get(DcMotorEx.class, "perpSlide");
        backSlide.setDirection(DcMotorEx.Direction.REVERSE);
        frontSlide.setDirection(DcMotorEx.Direction.REVERSE);
        perpSlide.setDirection(DcMotorEx.Direction.REVERSE);
        backSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        perpSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
            double controlLateral =  gamepad1.right_stick_x;
            double controlYaw =  gamepad1.left_stick_x;
            boolean controlSlow =  gamepad1.left_bumper;
            boolean controlReverse =  gamepad1.right_bumper;

            double controlBackSlide = -gamepad2.left_stick_y;
            double controlFrontSlide = -gamepad2.left_stick_y;
            double controlPerpSlide = gamepad2.left_stick_x;
            boolean controlIntake = gamepad2.left_bumper;
            boolean controlOuttake = gamepad2.right_bumper;
            boolean controlSlideOverride = gamepad2.dpad_up;
            boolean controlResetBackSlide = gamepad1.dpad_down && gamepad1.dpad_right || gamepad2.dpad_down && gamepad2.dpad_right;
            boolean controlResetFrontSlide = gamepad1.dpad_down && gamepad1.dpad_right || gamepad2.dpad_down && gamepad2.dpad_right;
            boolean controlResetPerpendicularSlide = gamepad1.dpad_down && gamepad1.dpad_left || gamepad2.dpad_down && gamepad2.dpad_left;


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

            if (controlIntake) {
                intakeA.setPower(0.2);
                intakeB.setPower(0.2);
            } else if (controlOuttake) {
                intakeA.setPower(-0.2);
                intakeB.setPower(-0.2);
            }

            backSlide.setPower(controlBackSlide);
            frontSlide.setPower(controlFrontSlide);
            perpSlide.setPower(controlPerpSlide);



            /* CONFIG BUTTONS */


            // Telemetry updates in case there is any telemetry that needs to be displayed.
            telemetry.update();
        }
    }}
