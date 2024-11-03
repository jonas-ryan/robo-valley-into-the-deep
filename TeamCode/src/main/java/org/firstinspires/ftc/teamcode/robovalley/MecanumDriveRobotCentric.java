package org.firstinspires.ftc.teamcode.robovalley;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Robot Centric", group="Mecanum Drive")
//@Disabled
public class MecanumDriveRobotCentric extends LinearOpMode {

    private double slowSpeed = 0.333;
    private double speedMultiplier = 1.0;
    private double slideSlowSpeed = 0.333;
    private double wormSlowSpeed = 0.333;
    private int slideSafetyMaximum = 4200;
    private int slideSafetyMinimum = 500;
    private double slideSafetySpeed = 0.2;

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
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawWrist = hardwareMap.get(CRServo.class, "intake");
        clawWrist.setDirection(DcMotorSimple.Direction.FORWARD);
        claw = hardwareMap.get(Servo.class, "claw");

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Robot Centric");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double controlAxial = -gamepad1.right_stick_y;
            double controlLateral =  gamepad1.right_stick_x;
            double controlYaw =  gamepad1.left_stick_x;
            boolean controlSlow =  gamepad1.left_bumper;
            boolean controlReverse =  gamepad1.right_bumper;

            double controlSlide =  -gamepad2.right_stick_y;
            double controlWorm = -gamepad2.left_stick_y;
            boolean controlSlideSlow =  gamepad2.left_bumper;
            double controlClawRotation =  gamepad2.right_trigger - gamepad2.left_trigger;
            boolean controlClawOpen = gamepad2.b;
            boolean controlClawClosed = gamepad2.a;

            // DRIVETRAIN BLOCK

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
            // if (slide > 0.8) {
            //     slide = slide * 5 - 3.8;
            // } else if (slide < 0.8) {
            //     slide = slide * 5 + 3.8;
            // } else {
            //     slide = 0.4 * slide;
            // }

            // Reduces slide and worm gear speeds if the slow button is held.
            if (controlSlideSlow) {
                controlSlide *= slideSlowSpeed;
                controlWorm *= wormSlowSpeed;
            }
            // Checks if the linear slide is past the maximum and then if power is trying to be applied in that direction, set it to zero.
            if (linearSlide.getCurrentPosition() > slideSafetyMaximum && controlSlide > 0) {
                controlSlide = 0;
            }
            // Checks if the linear slide is below a certain threshold and then limits the speed of the motor if it is being powered towards the starting position.
            if (linearSlide.getCurrentPosition() < slideSafetyMinimum && controlSlide < 0) {
                controlSlide *= slideSafetySpeed;
            }
            // Sets the powers of the slide and worm gear.
            linearSlide.setPower(controlSlide);
            wormGear.setPower(controlWorm);

            // Checks if the claw rotation buttons are held and then rotates the servo accordingly.
            clawWrist.setPower(controlClawRotation);

            if (controlClawOpen) {
                claw.setPosition(0.25);
            } else if (controlClawClosed) {
                claw.setPosition(0);
            }

            // Telemetry updates in case there is any telemetry that needs to be displayed.
            telemetry.update();
        }
    }}
