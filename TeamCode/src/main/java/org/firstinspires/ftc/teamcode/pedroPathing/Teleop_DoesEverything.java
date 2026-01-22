package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Teleop_DoesEverything", group="Linear OpMode")
public class Teleop_DoesEverything extends LinearOpMode {

    // Replaced IMU with Pinpoint as the angle source
    private GoBildaPinpointDriver pinpoint;

    private ElapsedTime runtime = new ElapsedTime();

    // NEW: event timing + persistent log
    private final ElapsedTime eventTimer = new ElapsedTime();
    private double lastEventMs = 0.0;
    private final List<String> eventLog = new ArrayList<>();
    private static final int MAX_LOG_LINES = 20; // keep screen readable

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shootLeft = null;
    private DcMotor shootRight = null;
    private DcMotor intake = null;
    private DcMotor guider = null;
    private Servo s1 = null;
    private CRServo s2 = null;
    private Servo s3 = null;
    private CRServo s4 = null;
    private CRServo s5 = null;
    private CRServo s6 = null;

    double left1Y, right1Y, left1X, right1X;
    double left2Y, right2Y, left2X, right2X;
    boolean flag_correction = true;
    double blPower, flPower, brPower, frPower;
    static final double DEADZONE = 0.1;

    // Intake toggle state
    boolean intakeprevious = false;
    boolean intakeConstant = false;

    // Shooter states
    boolean ShootPrevious = false;
    boolean ShootConstant = false;      // B toggle reverse
    boolean aPrev = false;              // for A press/release detection

    // Guider state: -1 reverse, 0 off, +1 forward
    int guiderState = 0;       // current
    int guiderPrevState = 0;   // previous

    private double clampDeadzone(double val) {
        return (Math.abs(val) < DEADZONE) ? 0 : val;
    }

    // NEW: Add a log line with Δt since last event (global)
    private void logEvent(String msg) {
        double now = eventTimer.milliseconds();
        long delta = Math.round(now - lastEventMs);
        lastEventMs = now;

        // store "(delta ms)" exactly like you asked
        eventLog.add(msg + " (" + delta + " ms)");

        // prevent infinite growth (keep most recent MAX_LOG_LINES)
        while (eventLog.size() > MAX_LOG_LINES) {
            eventLog.remove(0);
        }
    }

    // NEW: Render the persistent log every loop without clearing it
    private void showLog() {
        telemetry.addLine("=== Event Log (Δt since last event) ===");
        for (String line : eventLog) {
            telemetry.addLine(line);
        }
    }

    @Override
    public void runOpMode() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.resetPosAndIMU();

        telemetry.addData("Gyro Status", "Pinpoint Initialized");
        telemetry.update();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");
        intake = hardwareMap.get(DcMotor.class,"intake");
        shootLeft = hardwareMap.get(DcMotor.class,"sLeft");
        shootRight = hardwareMap.get(DcMotor.class,"sRight");
        guider = hardwareMap.get(DcMotor.class,"guide");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(CRServo.class, "s4");
        s5 = hardwareMap.get(CRServo.class,"s5");
        s6 = hardwareMap.get(CRServo.class,"s6");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        eventTimer.reset();
        lastEventMs = 0.0;
        eventLog.clear();
        logEvent("OpMode started"); // first line, delta will be ~0ms

        while (opModeIsActive()) {

            left1Y = clampDeadzone(gamepad1.left_stick_y * -1);
            left1X = clampDeadzone(gamepad1.left_stick_x);
            right1Y = clampDeadzone(gamepad1.right_stick_y * -1);
            right1X = clampDeadzone(gamepad1.right_stick_x);

            left2Y = clampDeadzone(gamepad2.left_stick_y * -1);
            left2X = clampDeadzone(gamepad2.left_stick_x);
            right2Y = clampDeadzone(gamepad2.right_stick_y * -1);
            right2X = clampDeadzone(gamepad2.right_stick_x);

            // keep sorter zero like you had
            s2.setPower(0);

            // s1 bumpers (no logging requested here)
            if (gamepad2.right_bumper) {
                s1.setDirection(Servo.Direction.FORWARD);
                s1.setPosition(0.5);
            }
            if (gamepad2.left_bumper) {
                s1.setDirection(Servo.Direction.REVERSE);
                s1.setPosition(-0.5);
            }

            // =========================
            // INTAKE TOGGLE (X)
            // =========================
            if (gamepad2.x && !intakeprevious) { // rising edge
                intakeConstant = !intakeConstant;
                if (intakeConstant) {
                    intake.setPower(1);
                    logEvent("Intake Activated (Toggled)");
                } else {
                    intake.setPower(0);
                    logEvent("Intake Deactivated");
                }
            }
            intakeprevious = gamepad2.x;

            // If you ONLY want toggle control for intake, do NOT override power elsewhere.
            // (This preserves your current behavior: intake power only changes on toggles.)

            // =========================
            // SHOOTER
            // B = toggle reverse
            // A = hold forward override (cancels toggle)
            // =========================

            // Rising edge for B toggle (ONLY if A is not currently held)
            if (gamepad2.b && !ShootPrevious && !gamepad2.a) {
                ShootConstant = !ShootConstant;
                if (ShootConstant) {
                    logEvent("Shooter Activated (Toggle Reverse)");
                } else {
                    logEvent("Shooter Deactivated (Toggle Off)");
                }
            }
            ShootPrevious = gamepad2.b;

            // Detect A press/release for logging
            boolean aNow = gamepad2.a;
            if (aNow && !aPrev) {
                // A pressed
                if (ShootConstant) {
                    ShootConstant = false; // cancel toggle
                    logEvent("Shooter Toggle Canceled by A");
                }
                logEvent("Shooter Forward Override (A) ON");
            } else if (!aNow && aPrev) {
                // A released
                logEvent("Shooter Forward Override (A) OFF");
            }
            aPrev = aNow;

            // Apply shooter motor powers (no spam logs here)
            if (gamepad2.a) {
                // forward override
                shootRight.setPower(1);
                shootLeft.setPower(-1);
            } else if (ShootConstant) {
                // reverse toggle (your chosen power)
                shootRight.setPower(-0.8);
                shootLeft.setPower(0.8);
            } else {
                shootRight.setPower(0);
                shootLeft.setPower(0);
            }

            // =========================
            // GUIDER
            // Only log on transitions (OFF<->ON / direction changes)
            // =========================
            int desiredGuiderState;
            boolean right = gamepad2.dpad_right;
            boolean left = gamepad2.dpad_left;

            if (right && !left) desiredGuiderState = 1;
            else if (left && !right) desiredGuiderState = -1;
            else desiredGuiderState = 0; // both or neither -> off

            if (desiredGuiderState != guiderState) {
                guiderState = desiredGuiderState;

                if (guiderState == 1) {
                    logEvent("Guider ON (Forward)");
                } else if (guiderState == -1) {
                    logEvent("Guider ON (Reverse)");
                } else {
                    logEvent("Guider OFF");
                }
            }

            // Apply guider power (no spam logs)
            if (guiderState == 1) guider.setPower(1);
            else if (guiderState == -1) guider.setPower(-1);
            else guider.setPower(0);

            // =========================
            // other controls you already had
            // =========================
            if (gamepad2.dpad_up) {
                frontLeftDrive.setPower(0.3);
                frontRightDrive.setPower(-0.3);
                backRightDrive.setPower(-0.3);
                backLeftDrive.setPower(0.3);
            }

            if (gamepad2.dpad_down) {
                flPower = 0.3;
                blPower = 0.3;
                frPower = 0.3;
                brPower = 0.3;
            }

            if (gamepad2.right_trigger > 0.1) {
                s3.setPosition(0.5);
            }
            if (gamepad2.left_trigger > 0.1) {
                s3.setPosition(0.7);
            }

            // =========================
            // MOVEMENT (unchanged)
            // =========================
            boolean leftStickActive = (left2X != 0) || (left2Y != 0);
            boolean rightStickActive = right2X != 0;

            if (leftStickActive == rightStickActive) {
                flPower = 0;
                frPower = 0;
                brPower = 0;
                blPower = 0;

            } else if (leftStickActive) {

                double THRESH_WM_POWER = 0.8;
                double headingDeg = pinpoint.getYawScalar();
                double correction = headingDeg / 180.0;

                correction = (10.0 * correction * Math.abs(left2Y) / THRESH_WM_POWER);
                if (flag_correction == false) correction = 0;
                correction = 0;

                double maxPow = THRESH_WM_POWER;
                double flPow = left2Y + (left2X) + correction;
                maxPow = Math.max(maxPow, Math.abs(flPow));
                double blPow = left2Y - (left2X) + correction;
                maxPow = Math.max(maxPow, Math.abs(blPow));
                double frPow = left2Y - (left2X) - correction;
                maxPow = Math.max(maxPow, Math.abs(frPow));
                double brPow = left2Y + (left2X) - correction;
                maxPow = Math.max(maxPow, Math.abs(brPow));

                flPow = (flPow / maxPow) * THRESH_WM_POWER;
                blPow = (blPow / maxPow) * THRESH_WM_POWER;
                frPow = (frPow / maxPow) * THRESH_WM_POWER;
                brPow = (brPow / maxPow) * THRESH_WM_POWER;

                flPower = Range.clip(flPow, -THRESH_WM_POWER, THRESH_WM_POWER);
                blPower = Range.clip(blPow, -THRESH_WM_POWER, THRESH_WM_POWER);
                frPower = Range.clip(frPow, -THRESH_WM_POWER, THRESH_WM_POWER);
                brPower = Range.clip(brPow, -THRESH_WM_POWER, THRESH_WM_POWER);

            } else {
                double THRESH_WM_POWER_FORTURN = 0.8;
                flPower = Range.clip((right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
                blPower = Range.clip((right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
                frPower = Range.clip((-right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
                brPower = Range.clip((-right2X), -THRESH_WM_POWER_FORTURN, THRESH_WM_POWER_FORTURN);
                idle();
            }

            backLeftDrive.setPower(blPower);
            backRightDrive.setPower(brPower);
            frontLeftDrive.setPower(flPower);
            frontRightDrive.setPower(frPower);

            // =========================
            // TELEMETRY OUTPUT
            // =========================

            // persistent event log
            showLog();

            telemetry.update();
        }
    }
}
