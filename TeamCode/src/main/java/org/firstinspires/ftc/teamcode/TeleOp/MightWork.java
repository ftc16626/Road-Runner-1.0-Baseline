package org.firstinspires.ftc.teamcode.TeleOp;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@TeleOp(name = "mightwork", group = "robot")
public class MightWork extends LinearOpMode {

    // Drive
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;

    // Shooter motors (three)
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx shooter3;

    // Servos
    private Servo leftHoodServo;
    private Servo rightHoodServo;
    private CRServo rollerServo;
    private Servo flipper1;
    private Servo flipper2;
    private Servo flipper3;

    // Color sensors
    private NormalizedColorSensor first;
    private NormalizedColorSensor second;
    private NormalizedColorSensor third;
    private NormalizedColorSensor fourth;
    private NormalizedColorSensor fifth;
    private NormalizedColorSensor sixth;

    // Vision
    private VisionPortal allSeeingEye;
    private AprilTagProcessor aprilTag;

    // PID controllers (one per motor)
    private ShooterPID pid1;
    private ShooterPID pid2;
    private ShooterPID pid3;

    // Constants / tuning
    private static final double TICKS_PER_REV = 28.0;
    private double targetRPM = 2300.0;             // default
    private double hoodPositionClose = 0.225;      // safe hood positions
    private double hoodPositionFar = 0.40;         // do not exceed hardware limits
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Telemetry control
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL = 0.20; // sec (200ms)

    // Misc
    private View relativeLayout;

    @Override
    public void runOpMode() {

        initHardware();
        initAprilTag();

        telemetry.addData("Status", "Ready - press START");
        telemetry.update();

        // Wait for start
        waitForStart();

        // Reset timers for PID sampling
        pid1.resetSampler();
        pid2.resetSampler();
        pid3.resetSampler();
        loopTimer.reset();
        telemetryTimer.reset();

        // Main loop
        while (opModeIsActive()) {
            double dtLoop = loopTimer.seconds();
            loopTimer.reset();

            // --- DRIVE (mecanum style) ---
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double lf = drive + turn - strafe;
            double lb = drive + turn + strafe;
            double rf = drive - turn + strafe;
            double rb = drive - turn - strafe;

            double denom = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1.0);

            leftFrontMotor.setPower(lf / denom);
            leftBackMotor.setPower(lb / denom);
            rightFrontMotor.setPower(rf / denom);
            rightBackMotor.setPower(rb / denom);

            // --- RPM mode selection ---
            if (gamepad2.dpad_left) {
                targetRPM = 2300.0;
            } else if (gamepad2.dpad_right) {
                targetRPM = 3000.0;
            }

            double hoodPos = (gamepad2.dpad_left) ? hoodPositionClose : hoodPositionFar;

            // --- Shooter control ---
            if (gamepad2.right_bumper) {
                // set hoods (mirror)
                leftHoodServo.setPosition(hoodPos);
                rightHoodServo.setPosition(hoodPos);

                // update independent PIDs -- pass gamepad2 for rumble
                pid1.update(targetRPM, gamepad2);
                pid2.update(targetRPM, gamepad2);
                pid3.update(targetRPM, gamepad2);

            } else if (gamepad2.left_bumper) {
                // reverse to un-jam
                setAllShooterPower(-0.25);
                // keep flippers in a position to clear jam
                flipper1.setPosition(0.6);
                flipper2.setPosition(0.6);
                flipper3.setPosition(0.52);

                // reset PID internal integrators so we don't wind up
                pid1.resetIntegral();
                pid2.resetIntegral();
                pid3.resetIntegral();

            } else {
                // idle shooters
                setAllShooterPower(0.0);
                pid1.resetIntegral();
                pid2.resetIntegral();
                pid3.resetIntegral();
                pid1.resetRumble();
                pid2.resetRumble();
                pid3.resetRumble();
            }

            // --- Intake / roller (gamepad1) ---
            if (gamepad1.right_bumper) {
                rollerServo.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                rollerServo.setPower(-1.0);
            } else {
                rollerServo.setPower(0.0);
            }

            // --- Hood manual override (gamepad1) ---
            if (gamepad1.dpad_up) {
                rightHoodServo.setPosition(hoodPositionClose);
                leftHoodServo.setPosition(hoodPositionClose);
            } else if (gamepad1.dpad_down) {
                rightHoodServo.setPosition(0.0);
                leftHoodServo.setPosition(0.0);
            }

            // --- Flippers quick triggers (gamepad2) ---
            if (gamepad2.circle) {
                pulse(flipper3, 0.1, 0.53);
            }
            if (gamepad2.a) {
                pulse(flipper2, 0.9, 0.47);
            }
            if (gamepad2.square) {
                pulse(flipper1, 0.9, 0.47);
            }

            if (gamepad2.triangle) {
                flipper1.setPosition(0.53);
                flipper2.setPosition(0.53);
                flipper3.setPosition(0.49);
            }

            if (gamepad2.dpad_down) {
                flipper1.setPosition(0.47);
                flipper2.setPosition(0.47);
                flipper3.setPosition(0.53);
            }

            // --- Read AprilTag on demand (gamepad1.square) ---
            if (gamepad1.square && aprilTag != null) {
                List<AprilTagDetection> dets = aprilTag.getDetections();
                if (!dets.isEmpty()) {
                    for (AprilTagDetection d : dets) {
                        if (d.metadata != null) {
                            telemetry.addData("TagID", d.id);
                            telemetry.addData("Pose", "%.2f, %.2f, %.2f", d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
                        }
                    }
                } else {
                    telemetry.addData("AprilTag", "Not detected");
                }
            }

            // --- Telemetry (throttled) ---
            if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL) {
                telemetryTimer.reset();
                telemetry.addData("TargetRPM", "%d", (int) targetRPM);
                telemetry.addData("RPM1", "%d", (int) pid1.getRPM());
                telemetry.addData("RPM2", "%d", (int) pid2.getRPM());
                telemetry.addData("RPM3", "%d", (int) pid3.getRPM());
                telemetry.addData("ShooterPower1", "%.3f", shooter1.getPower());
                telemetry.addData("ShooterPower2", "%.3f", shooter2.getPower());
                telemetry.addData("ShooterPower3", "%.3f", shooter3.getPower());
                telemetry.update();
            }

            // small sleep to yield CPU; keep low for responsiveness
            sleep(15);
        }

        // cleanup on stop
        if (allSeeingEye != null) allSeeingEye.close();
        setAllShooterPower(0.0);
    }

    // -------------------- helpers --------------------

    private void initHardware() {
        // layout reference (optional)
        int relativeLayoutId = hardwareMap.appContext.getResources()
                .getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        try {
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        } catch (Exception ignored) {
        }

        // drive
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // shooters (three required)
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // servos and actuators
        rollerServo = hardwareMap.get(CRServo.class, "roller");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        flipper1 = hardwareMap.get(Servo.class, "flipper1");
        flipper2 = hardwareMap.get(Servo.class, "flipper2");
        flipper3 = hardwareMap.get(Servo.class, "flipper3");

        // color sensors
        first = safeGetColor("first");
        second = safeGetColor("second");
        third = safeGetColor("third");
        fourth = safeGetColor("fourth");
        fifth = safeGetColor("fifth");
        sixth = safeGetColor("sixth");

        enableSensorLight(first);
        enableSensorLight(second);
        enableSensorLight(third);
        enableSensorLight(fourth);
        enableSensorLight(fifth);
        enableSensorLight(sixth);

        // Create PID objects for each shooter motor
        pid1 = new ShooterPID(shooter1);
        pid2 = new ShooterPID(shooter2);
        pid3 = new ShooterPID(shooter3);
    }

    private NormalizedColorSensor safeGetColor(String name) {
        try {
            return hardwareMap.get(NormalizedColorSensor.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private void enableSensorLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) {
            try {
                ((SwitchableLight) s).enableLight(true);
            } catch (Exception ignored) {
            }
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        try {
            allSeeingEye = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "allSeeingEye"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            allSeeingEye = null;
        }
    }

    private void setAllShooterPower(double p) {
        shooter1.setPower(p);
        shooter2.setPower(p);
        shooter3.setPower(p);
    }

    private void pulse(Servo s, double a, double b) {
        s.setPosition(a);
        sleep(100);
        s.setPosition(b);
    }

    // ------------------ Shooter PID inner class ------------------
    private static class ShooterPID {
        private final DcMotorEx motor;
        private final ElapsedTime timer = new ElapsedTime();

        // PID gains (conservative defaults; tune on robot)
        private double Kp = 0.0012;
        private double Ki = 0.0;
        private double Kd = 0.0005;

        private double integral = 0.0;
        private double lastError = 0.0;
        private double integralLimit = 2000.0;

        // encoder sampling
        private int lastPos;
        private long lastTimeNano;

        // rumble/stability
        private boolean hasRumbled = false;
        private double stableTimer = 0.0;
        private final double RPM_TOL = 60.0;
        private final double STABLE_REQUIRED = 0.25;

        public ShooterPID(DcMotorEx motor) {
            this.motor = motor;
            this.lastPos = motor.getCurrentPosition();
            this.lastTimeNano = System.nanoTime();
            timer.reset();
        }

        public void resetSampler() {
            lastPos = motor.getCurrentPosition();
            lastTimeNano = System.nanoTime();
            timer.reset();
        }

        public void resetIntegral() {
            integral = 0.0;
            lastError = 0.0;
        }

        public void resetRumble() {
            hasRumbled = false;
            stableTimer = 0.0;
        }

        public double getRPM() {
            int curPos = motor.getCurrentPosition();
            long curTime = System.nanoTime();

            int deltaPos = curPos - lastPos;
            long deltaNano = curTime - lastTimeNano;
            if (deltaNano <= 0) deltaNano = 1;

            double seconds = deltaNano / 1e9;
            double ticksPerSec = deltaPos / seconds;
            double rpm = (ticksPerSec / TICKS_PER_REV) * 60.0;

            lastPos = curPos;
            lastTimeNano = curTime;

            return Math.abs(rpm);
        }

        public void update(double targetRPM, com.qualcomm.robotcore.hardware.Gamepad gp) {
            double currentRPM = getRPM();

            double dt = timer.seconds();
            timer.reset();
            if (dt <= 0) dt = 0.001;

            double error = targetRPM - currentRPM;

            // integral with anti-windup
            integral += error * dt;
            if (integral > integralLimit) integral = integralLimit;
            if (integral < -integralLimit) integral = -integralLimit;

            double derivative = (error - lastError) / dt;
            lastError = error;

            double out = Kp * error + Ki * integral + Kd * derivative;
            // clamp to [0,1] for forward; negative handled elsewhere (left bumper)
            if (out < 0.0) out = 0.0;
            if (out > 1.0) out = 1.0;

            motor.setPower(out);

            // rumble when stable
            if (Math.abs(currentRPM - targetRPM) <= RPM_TOL) {
                stableTimer += dt;
            } else {
                stableTimer = 0.0;
                hasRumbled = false;
            }

            if (!hasRumbled && stableTimer >= STABLE_REQUIRED) {
                // rumble the operator gamepad (the caller should pass the operator gamepad)
                gp.rumble(0.7, 0.7, 300);
                hasRumbled = true;
            }
        }
    }
}

