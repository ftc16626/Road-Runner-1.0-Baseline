package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.view.View;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous(name="TheRedUpAgainstWallAutoYouShouldUse", group="Robot")
public class AutoTest extends LinearOpMode {

    /* ---------- Hardware ---------- */
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx shooter1, shooter2, shooter3;
    private CRServo intakeServo;
    private Servo rightHoodServo, leftHoodServo;
    private NormalizedColorSensor colorSensorI, colorSensorII, colorSensorIII,
            colorSensorIV, colorSensorV, colorSensorVI;
    private VisionPortal allSeeingEye;
    private AprilTagProcessor aprilTag;
    private Servo servoI, servoII, servoIII;

    /* ---------- Constants & State ---------- */
    // SHOOTER target RPM (set to requested 2300)
    private final double TARGET_RPM = 2300.0;
    private final double SHOOTER_TICKS_PER_REV = 28.0; // REV encoder CPR

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    // PID controllers (one per shooter)
    private PIDControl pid1, pid2, pid3;

    // Drive constants (unchanged)
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // artifact pattern from AprilTag during INIT
    private double artifactPattern = 0;

    // simple done flag for firing action
    private double done = 0;

    /* ---------- Shooter Helper Class (firing sequence) ---------- */
    public class ShooterActions {
        public Action Fire() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime localTimer;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        localTimer = new ElapsedTime();
                        localTimer.reset();
                        initialized = true;

                        // sequence simplified to respect artifactPattern mapping from init
                        // using servo positions from your original code
                        new Thread(() -> {
                            try {
                                // small delays between flips (timings preserved roughly)
                                Thread.sleep(4000);
                                if (artifactPattern == 22) servoI.setPosition(0.47);
                                else servoII.setPosition(0.47);
                                Thread.sleep(500);
                                if (artifactPattern == 22) servoI.setPosition(0.9);
                                else servoII.setPosition(0.9);
                                Thread.sleep(500);
                                servoIII.setPosition(0.53);
                                Thread.sleep(500);
                                servoIII.setPosition(0.1);
                                done = 1;
                            } catch (InterruptedException e) {
                                Thread.currentThread().interrupt();
                            }
                        }).start();
                    }

                    // keep action alive briefly to ensure sequence completion by timeline
                    return localTimer.seconds() < 2.5;
                }
            };
        }

        public Action intake() {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime t;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        encoderDrive(0, 0, 0, 0, 15, false, -1, 0.15, 5);
                        t = new ElapsedTime();
                        initialized = true;
                    }
                    return t.seconds() < 2;
                }
            };
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // RoadRunner start pose preserved
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(3.5592));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // init hardware
        initHardware();

        // init apriltag/vision
        initAprilTag();

        telemetry.addLine("Scanning for AprilTag during INIT...");
        telemetry.update();

        // APRILTAG INIT LOOP (runs until start pressed)
        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection d = detections.get(0);
                artifactPattern = d.id;
                // keep your remapping logic exactly
                if (artifactPattern == 23) artifactPattern = 21;
                else if (artifactPattern == 21) artifactPattern = 22;
                else if (artifactPattern == 22) artifactPattern = 23;

                telemetry.addData("AprilTag detected (raw id)", d.id);
                telemetry.addData("artifactPattern (mapped)", artifactPattern);
            } else {
                telemetry.addData("AprilTag", "none");
            }
            telemetry.update();
            sleep(50);
        }

        // WAIT FOR MATCH START
        waitForStart();

        // Close camera to free resources
        if (allSeeingEye != null) {
            try { allSeeingEye.close(); } catch (Exception ignored) {}
            allSeeingEye = null;
        }

        // initialize PID controllers AFTER start (each with own motor)
        pid1 = new PIDControl(shooter1, SHOOTER_TICKS_PER_REV);
        pid2 = new PIDControl(shooter2, SHOOTER_TICKS_PER_REV);
        pid3 = new PIDControl(shooter3, SHOOTER_TICKS_PER_REV);

        // start shooter PID thread (runs for 30 seconds or until opMode stops)
        shooterTimer.reset();
        Thread shooterThread = new Thread(() -> {
            final long SAMPLE_MS = 20;
            while (opModeIsActive() && shooterTimer.seconds() < 30.0) {
                try {
                    double power1 = pid1.update(TARGET_RPM);
                    double power2 = pid2.update(TARGET_RPM);
                    double power3 = pid3.update(TARGET_RPM);

                    shooter1.setPower(power1);
                    shooter2.setPower(power2);
                    shooter3.setPower(power3);

                    Thread.sleep(SAMPLE_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception ex) {
                    // safe shutdown if unexpected error occurs
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                    shooter3.setPower(0);
                    break;
                }
            }
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooter3.setPower(0);
        }, "ShooterThread");
        shooterThread.setDaemon(true);
        shooterThread.start();

        // pre-trajectory hood positions
        leftHoodServo.setPosition(0);
        rightHoodServo.setPosition(0);

        // Build your RoadRunner trajectories (preserved from original)
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-46.5281, -19.4465))
                .turn(Math.toRadians(-41.69))
                .waitSeconds(3);

        Pose2d newPose = new Pose2d(9.9556, -30.3882, Math.toRadians(-95.2059));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(newPose)
                .lineToY(-32.1531)
                .waitSeconds(3);

        Pose2d new2Pose = new Pose2d(9.9556, -44.4149, Math.toRadians(-95.2059));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new2Pose)
                .strafeTo(new Vector2d(0, 0))
                .turn(-53.9244)
                .waitSeconds(3);

        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(9.9556, -30.3882))
                .build();

        // choose a trajectory (kept original flow)
        int position = 1;
        Action chosen;
        if (position == 1) chosen = tab1.build();
        else if (position == 2) chosen = tab2.build();
        else chosen = tab3.build();

        ShooterActions shoot = new ShooterActions();

        // Run route and intake as before
        Actions.runBlocking(new SequentialAction(chosen));
        chosen = (position == 1) ? tab1.build() : (position == 2) ? tab2.build() : tab3.build();
        Actions.runBlocking(new SequentialAction(shoot.intake(), chosen));
        chosen = (position == 1) ? tab1.build() : (position == 2) ? tab2.build() : tab3.build();
        Actions.runBlocking(new SequentialAction(chosen));
        Actions.runBlocking(new SequentialAction(trajectoryActionCloseOut));

        // BEFORE firing, wait a short timeout for shooter RPMs to come up and only fire if within tolerance
        double waitStart = getRuntime();
        double waitTimeout = 5.0; // seconds to wait for reaching speed
        boolean allAtTarget = false;
        double tolerance = 50.0; // rpm tolerance

        while (opModeIsActive() && (getRuntime() - waitStart) < waitTimeout) {
            double r1 = pid1.peekRPM();
            double r2 = pid2.peekRPM();
            double r3 = pid3.peekRPM();
            telemetry.addData("RPMs", "%4.0f, %4.0f, %4.0f", r1, r2, r3);
            telemetry.update();
            if (r1 >= TARGET_RPM - tolerance && r2 >= TARGET_RPM - tolerance && r3 >= TARGET_RPM - tolerance) {
                allAtTarget = true;
                break;
            }
            sleep(50);
        }

        if (opModeIsActive() && shooterTimer.seconds() < 30.0 && allAtTarget) {
            Actions.runBlocking(new SequentialAction(shoot.Fire()));
        } else {
            telemetry.addData("Fire", "Skipped: shooters not at speed or timer expired");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /* ---------- Hardware init ---------- */
    private void initHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBMotor");

        intakeServo = hardwareMap.get(CRServo.class, "roller");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");

        servoI = hardwareMap.get(Servo.class, "flipper1");
        servoII = hardwareMap.get(Servo.class, "flipper2");
        servoIII = hardwareMap.get(Servo.class, "flipper3");

        colorSensorI = hardwareMap.get(NormalizedColorSensor.class, "first");
        colorSensorII = hardwareMap.get(NormalizedColorSensor.class, "second");
        colorSensorIII = hardwareMap.get(NormalizedColorSensor.class, "third");
        colorSensorIV = hardwareMap.get(NormalizedColorSensor.class, "fourth");
        colorSensorV = hardwareMap.get(NormalizedColorSensor.class, "fifth");
        colorSensorVI = hardwareMap.get(NormalizedColorSensor.class, "sixth");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");

        // motor directions (kept as original)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter3.setDirection(DcMotorEx.Direction.FORWARD);

        leftHoodServo.setDirection(Servo.Direction.REVERSE);

        // encoder modes
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // enable color sensor lights if available
        if (colorSensorI instanceof SwitchableLight) ((SwitchableLight)colorSensorI).enableLight(true);
        if (colorSensorII instanceof SwitchableLight) ((SwitchableLight)colorSensorII).enableLight(true);
        if (colorSensorIII instanceof SwitchableLight) ((SwitchableLight)colorSensorIII).enableLight(true);
        if (colorSensorIV instanceof SwitchableLight) ((SwitchableLight)colorSensorIV).enableLight(true);
        if (colorSensorV instanceof SwitchableLight) ((SwitchableLight)colorSensorV).enableLight(true);
        if (colorSensorVI instanceof SwitchableLight) ((SwitchableLight)colorSensorVI).enableLight(true);

        colorSensorI.setGain(1);
        colorSensorII.setGain(1);
        colorSensorIII.setGain(1);
        colorSensorIV.setGain(1);
        colorSensorV.setGain(1);
        colorSensorVI.setGain(1);

        // starting servo positions
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);
    }

    /* ---------- Vision / AprilTag ---------- */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        allSeeingEye = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "allSeeingEye"))
                .addProcessor(aprilTag)
                .build();
    }

    /* ---------- Encoder-drive helper (kept behavior, cleaned) ---------- */
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             boolean strafe, double IntakePower, double Angulinator,
                             double timeoutS) {

        if (!opModeIsActive()) return;

        // adjust motor directions for strafing vs forward/back
        if (strafe) {
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        }

        int newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
        int newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
        int newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
        int newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

        leftFrontDrive.setTargetPosition(newLeftFrontTarget);
        rightFrontDrive.setTargetPosition(newRightFrontTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftFrontDrive.setPower(Math.abs(speed));
        rightFrontDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        // Loop while motors are busy and timeout hasn't elapsed
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            leftHoodServo.setPosition(Angulinator);
            rightHoodServo.setPosition(Angulinator);
            intakeServo.setPower(IntakePower);

            // Read color sensors fresh every loop
            NormalizedRGBA colorsI = colorSensorI.getNormalizedColors();
            NormalizedRGBA colorsII = colorSensorII.getNormalizedColors();
            NormalizedRGBA colorsIII = colorSensorIII.getNormalizedColors();
            NormalizedRGBA colorsIV = colorSensorIV.getNormalizedColors();
            NormalizedRGBA colorsV = colorSensorV.getNormalizedColors();
            NormalizedRGBA colorsVI = colorSensorVI.getNormalizedColors();

            String ColorI = (colorsI.green > colorsI.blue || colorsII.green > colorsII.blue) ? "Green" : "Purple";
            String ColorII = (colorsIII.green > colorsIII.blue || colorsIV.green > colorsIV.blue) ? "Green" : "Purple";
            String ColorIII = ((colorsV.green > 0.5 && colorsV.blue < 0.5) || (colorsVI.green > 0.5 && colorsVI.blue < 0.5)) ? "Green" : "Purple";

            telemetry.addData("Running to", " LF:%7d RF:%7d LB:%7d RB:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
            telemetry.addData("Currently at", "LF:%7d RF:%7d LB:%7d RB:%7d",
                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.addLine().addData("Tag ID:", artifactPattern);
            telemetry.addLine().addData("ColorI:", ColorI);
            telemetry.addLine()
                    .addData("Red", colorsI.red)
                    .addData("Green", colorsI.green)
                    .addData("Blue", colorsI.blue);
            telemetry.update();
        }

        // stop all drive motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // reset servos to safe positions
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }

    /* ===========================
       CLEAN PIDControl (drop-in)
       =========================== */
    private static class PIDControl {
        // PID gains (tweak these if needed)
        private double Kp = 8.0;
        private double Ki = 0.5;
        private double Kd = 1.3;

        // motor reference
        private final DcMotorEx motor;

        // state
        private double integral = 0.0;
        private double lastError = 0.0;

        // integral clamp
        private final double integralLimit = 2000.0;

        // sampling state (per motor)
        private int lastPos;
        private long lastTimeNs;
        private double lastRPM = 0.0;

        // ticks per revolution for this motor (pass in via constructor)
        private final double ticksPerRev;

        public PIDControl(DcMotorEx motor, double ticksPerRev) {
            this.motor = motor;
            this.ticksPerRev = ticksPerRev;
            this.lastPos = motor.getCurrentPosition();
            this.lastTimeNs = System.nanoTime();
        }

        // Returns most recent RPM sample; also updates internal sampler
        public synchronized double peekRPM() {
            // compute but do not reset PID internal timers
            int curPos = motor.getCurrentPosition();
            long now = System.nanoTime();

            int dPos = curPos - lastPos;
            long dNs = now - lastTimeNs;
            if (dNs <= 0) dNs = 1;

            double dt = dNs / 1e9;
            double ticksPerSec = dPos / dt;
            double rpm = (ticksPerSec / ticksPerRev) * 60.0;

            // smooth small jitter by exponential smoothing
            lastRPM = 0.7 * lastRPM + 0.3 * Math.abs(rpm);

            // do not update lastPos/time here — leave that to getRPM()/update sampling
            return Math.abs(lastRPM);
        }

        // update RPM sample and compute PID output (power 0..1)
        public synchronized double update(double targetRPM) {
            // compute current RPM based on delta since last sample
            int curPos = motor.getCurrentPosition();
            long now = System.nanoTime();

            int dPos = curPos - lastPos;
            long dNs = now - lastTimeNs;
            if (dNs <= 0) dNs = 1;

            double dt = dNs / 1e9;
            if (dt < 1e-4) dt = 1e-4; // protect dt

            double ticksPerSec = dPos / dt;
            double currentRPM = (ticksPerSec / ticksPerRev) * 60.0;
            currentRPM = Math.abs(currentRPM);

            // update sampler state
            lastPos = curPos;
            lastTimeNs = now;
            lastRPM = 0.7 * lastRPM + 0.3 * currentRPM;

            // PID calculations
            double error = targetRPM - lastRPM;

            // integral with clamp
            integral += error * dt;
            if (integral > integralLimit) integral = integralLimit;
            if (integral < -integralLimit) integral = -integralLimit;

            double derivative = (error - lastError) / dt;

            // reduce derivative spikes
            if (Math.abs(derivative) > 5000.0) derivative = 0.0;

            lastError = error;

            double output = Kp * error + Ki * integral + Kd * derivative;

            // clamp to motor power range
            if (output < 0.0) output = 0.0;
            if (output > 1.0) output = 1.0;

            return output;
        }

        // convenience for external telemetry
        public synchronized double getLastRPM() {
            return lastRPM;
        }
    }
}