// (Full optimized OpMode - drop-in replacement)
package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.ComponentSubClasses.DriveSubsystemOdometryReady.TICKS_PER_REV;

import android.app.Activity;
import android.view.View;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
@Disabled
public class Doubtful extends LinearOpMode {

    /* Hardware */
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx shooter3;

    private CRServo intakeServo;
    private Servo rightHoodServo;
    private Servo leftHoodServo;
    private NormalizedColorSensor colorSensorI;
    private NormalizedColorSensor colorSensorII;
    private NormalizedColorSensor colorSensorIII;
    private NormalizedColorSensor colorSensorIV;
    private NormalizedColorSensor colorSensorV;
    private NormalizedColorSensor colorSensorVI;
    private VisionPortal allSeeingEye;
    private AprilTagProcessor aprilTag;

    private Servo servoI;
    private Servo servoII;
    private Servo servoIII;

    /* State & constants */
    public double targetRPM = 2150;
    public double ticksPerRevolution = 28;
    public double targetVelocity = (targetRPM / 60.0) * ticksPerRevolution;

    double artifactPattern = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    // PID controllers (one per shooter) - KEEP PID constants exactly
    private PIDControl pid1;
    private PIDControl pid2;
    private PIDControl pid3;

    // Encoder / drive constants (unchanged)
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    View relativeLayout;

    enum State {
        Get_To_Power,
        Fling,
        Finished
    }
    State state = State.Get_To_Power;

    private double done = 0;
    private double position = 0;

    public class Shooter {
        ElapsedTime timer;
        public Shooter(HardwareMap hw) {
            // constructor kept minimal; hardware already initialized in outer scope
        }

        // Fire action (kept identical timing/positions)
        public Action Fire() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        if (artifactPattern == 21) {
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000) {
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500) {
                                servoI.setPosition(0.47);
                            }
                            servoI.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000) {
                                servoIII.setPosition(0.53);
                            }
                            servoIII.setPosition(0.1);
                            done = 1;
                        } else if (artifactPattern == 22) {
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000) {
                                servoI.setPosition(0.47);
                            }
                            servoI.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500) {
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000) {
                                servoIII.setPosition(0.51);
                            }
                            servoIII.setPosition(0.1);
                            done = 1;
                        } else if (artifactPattern == 23) {
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000) {
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500) {
                                servoI.setPosition(0.47);
                            }
                            servoIII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000) {
                                servoIII.setPosition(0.51);
                            }
                            servoI.setPosition(0.1);
                            done = 1;
                        } else {
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000) {
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500) {
                                servoI.setPosition(0.47);
                            }
                            servoI.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000) {
                                servoIII.setPosition(0.53);
                            }
                            servoIII.setPosition(0.1);
                            done = 1;
                        }
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 1;
                }
            };
        }

        public Action intake() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        encoderDrive(0,0,0,0,15,false,-1,0.15,5);
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 2;
                }
            };
        }
        // other actions preserved (goBack, turn, turn2, etc.) — omitted for brevity but available in original version
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(3.5592));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // init vision (we will scan in init loop)
        initAprilTag();

        // hardware init
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

        // directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter3.setDirection(DcMotorEx.Direction.FORWARD);

        leftHoodServo.setDirection(Servo.Direction.REVERSE);

        // encoders
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

        telemetry.addData("Starting at", "%7d : %7d : %7d : %7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        telemetry.update();

        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);

        // ---- APRILTAG INIT LOOP (lower CPU usage: 100ms sleep) ----
        telemetry.addLine("Scanning for AprilTag during INIT...");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection d = detections.get(0);
                artifactPattern = d.id;
                if (artifactPattern == 23) artifactPattern = 21;
                else if (artifactPattern == 21) artifactPattern = 22;
                else if (artifactPattern == 22) artifactPattern = 23;

                telemetry.addData("AprilTag raw id", d.id);
                telemetry.addData("artifactPattern (mapped)", artifactPattern);
            } else {
                telemetry.addData("AprilTag", "none");
            }
            telemetry.update();
            sleep(100); // throttle init scanning to reduce CPU usage
        }

        waitForStart();

        // Safely close the webcam after play starts
        if (allSeeingEye != null) {
            try { allSeeingEye.close(); } catch (Exception ignored) {}
            allSeeingEye = null;
        }

        // init PIDs after start
        pid1 = new PIDControl(shooter1);
        pid2 = new PIDControl(shooter2);
        pid3 = new PIDControl(shooter3);

        // shooter thread: sample every 25 ms
        shooterTimer.reset();
        Thread shooterThread = new Thread(() -> {
            while (opModeIsActive() && shooterTimer.seconds() < 30.0) {
                try {
                    double out1 = pid1.update(targetRPM, null);
                    double out2 = pid2.update(targetRPM, null);
                    double out3 = pid3.update(targetRPM, null);

                    shooter1.setPower(out1);
                    shooter2.setPower(out2);
                    shooter3.setPower(out3);

                    Thread.sleep(25);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    shooter1.setPower(0);
                    shooter2.setPower(0);
                    shooter3.setPower(0);
                    break;
                }
            }
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooter3.setPower(0);
        });
        shooterThread.setDaemon(true);
        shooterThread.start();

        leftHoodServo.setPosition(0);
        rightHoodServo.setPosition(0);

        // RoadRunner trajectories preserved
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

        position = 1;
        Action trajectoryActionChosen;
        if (position == 1) trajectoryActionChosen = tab1.build();
        else if (position == 2) trajectoryActionChosen = tab2.build();
        else trajectoryActionChosen = tab3.build();

        Shooter shoot = new Shooter(hardwareMap);

        // Follow trajectories (unchanged flow)
        Actions.runBlocking(new SequentialAction(trajectoryActionChosen));

        position = 2;
        if (position == 1) trajectoryActionChosen = tab1.build();
        else if (position == 2) trajectoryActionChosen = tab2.build();
        else trajectoryActionChosen = tab3.build();
        Actions.runBlocking(new SequentialAction(shoot.intake(), trajectoryActionChosen));

        position = 3;
        if (position == 1) trajectoryActionChosen = tab1.build();
        else if (position == 2) trajectoryActionChosen = tab2.build();
        else trajectoryActionChosen = tab3.build();
        Actions.runBlocking(new SequentialAction(trajectoryActionChosen));

        Actions.runBlocking(new SequentialAction(trajectoryActionCloseOut));

        // --- WAIT for stable shooter RPMs BEFORE firing ---
        final double RPM_TOL = 30.0;          // tightened tolerance
        final int REQUIRED_STABLE = 3;        // require 3 consecutive stable samples
        int stableCount = 0;
        double waitStart = getRuntime();
        double waitTimeout = 5.0; // seconds
        boolean allAtTarget = false;
        long lastTelemetry = System.currentTimeMillis();

        while (opModeIsActive() && (getRuntime() - waitStart) < waitTimeout) {
            double r1 = pid1.getRPM();
            double r2 = pid2.getRPM();
            double r3 = pid3.getRPM();

            boolean r1ok = r1 >= targetRPM - RPM_TOL;
            boolean r2ok = r2 >= targetRPM - RPM_TOL;
            boolean r3ok = r3 >= targetRPM - RPM_TOL;

            if (r1ok && r2ok && r3ok) stableCount++;
            else stableCount = 0;

            if (stableCount >= REQUIRED_STABLE) {
                allAtTarget = true;
                break;
            }

            // throttle telemetry to ~200ms
            if (System.currentTimeMillis() - lastTelemetry > 200) {
                telemetry.addData("RPMs", "%4.0f, %4.0f, %4.0f (stable %d)", r1, r2, r3, stableCount);
                telemetry.update();
                lastTelemetry = System.currentTimeMillis();
            }
            sleep(50);
        }

        if (opModeIsActive() && shooterTimer.seconds() < 30.0 && allAtTarget) {
            Actions.runBlocking(new SequentialAction(shoot.Fire()));
        } else {
            telemetry.addData("Fire", "Skipped: shooters not stable or timer expired");
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Cleaned encoderDrive that preserves original behavior:
     * - updates hood servos & intake while moving
     * - reads color sensors each loop and reports them
     * - uses RUN_TO_POSITION and waits while isBusy
     */
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             boolean strafe, double IntakePower, double Angulinator,
                             double timeoutS) {

        if (!opModeIsActive()) return;

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

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            leftHoodServo.setPosition(Angulinator);
            rightHoodServo.setPosition(Angulinator);
            intakeServo.setPower(IntakePower);

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

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }

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

    // PID class (Kp/Ki/Kd unchanged)
    private static class PIDControl {
        private double Kp = 8;
        private double Ki = 0.5;
        private double Kd = 1.3;
        private final ElapsedTime timer1 = new ElapsedTime();
        private final DcMotorEx shooter1;

        private double integral1 = 0.0;
        private double lastError1 = 0.0;
        private double integralLimit1 = 2000.0;

        private int lastPos1;
        private long lastTimeNano1;

        public PIDControl(DcMotorEx motor) {
            this.shooter1 = motor;
            this.lastPos1 = motor.getCurrentPosition();
            this.lastTimeNano1 = System.nanoTime();
            timer1.reset();
        }

        public void resetSampler() {
            lastPos1 = shooter1.getCurrentPosition();
            lastTimeNano1 = System.nanoTime();
            timer1.reset();
        }

        public void resetIntegral() {
            integral1 = 0.0;
            lastError1 = 0.0;
        }

        public double getRPM() {
            int curPos = shooter1.getCurrentPosition();
            long curTime = System.nanoTime();

            int deltaPos = curPos - lastPos1;
            long deltaNano = curTime - lastTimeNano1;
            if (deltaNano <= 0) deltaNano = 1;

            double seconds = deltaNano / 1e9;
            double ticksPerSec = deltaPos / seconds;
            double rpm = (ticksPerSec / TICKS_PER_REV) * 60.0;

            lastPos1 = curPos;
            lastTimeNano1 = curTime;

            return Math.abs(rpm);
        }

        public double update(double targetRPM, com.qualcomm.robotcore.hardware.Gamepad gp) {
            double currentRPM = getRPM();

            double dt = timer1.seconds();
            timer1.reset();
            if (dt <= 0) dt = 0.001;

            double error = targetRPM - currentRPM;

            integral1 += error * dt;
            if (integral1 > integralLimit1) integral1 = integralLimit1;
            if (integral1 < -integralLimit1) integral1 = -integralLimit1;

            double derivative = (error - lastError1) / dt;
            lastError1 = error;

            double out = Kp * error + Ki * integral1 + Kd * derivative;
            if (out < 0.0) out = 0.0;
            if (out > 1.0) out = 1.0;

            return out;
        }
    }
}
