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

@Autonomous(name="RedUpAgainstTheGoalMeet3", group="Robot")
public class AutoTest extends LinearOpMode {

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
    private double currentVelocity;
    public double targetRPM = 2150;
    public double ticksPerRevolution = 28;
    public double targetVelocity = (targetRPM / 60.0) * ticksPerRevolution;

    double artifactPattern = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    // PID controllers (one per shooter) - keep PID constants as requested (A)
    private PIDControl pid1;
    private PIDControl pid2;
    private PIDControl pid3;

    // Encoder / drive constants (unchanged)
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    private final double SHOOTER_TICKS_PER_REV = 28.0;

    View relativeLayout;

    enum State {
        Get_To_Power,
        Fling,
        Finished
    }
    State state = State.Get_To_Power;

    /* --- Shooter actions & behavior kept largely as before, but cleaned --- */
    public class Shooter {
        private Servo sI;
        private Servo sII;
        private Servo sIII;
        ElapsedTime timer;

        public Shooter(HardwareMap hw) {
            // motors/servos already acquired in outer scope; keep this constructor lightweight
            sI = servoI;
            sII = servoII;
            sIII = servoIII;
        }

        // Fire action
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
                            // fallback identical to original default
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

        // intake movement action (kept behavior)
        public Action intake() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        encoderDrive(0, 0, 0, 0, 15, false, -1, 0.15, 5);
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 2;
                }
            };
        }
        public Action goBack() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        encoderDrive(0.2, -3.25, -3.25, -3.25, -3.25, false, 0, 0.225, 2);
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 2;
                }
            };
        }
        public Action turn() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        encoderDrive(0.25, 9.5, -9.5, 9.5, -9.5, false, 0, 0, 1);
                        encoderDrive(0.25, 2, 2, 2, 2, false, 0, 0, 1);
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 1;
                }
            };
        }
        public Action outOfShootingArea() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        encoderDrive(0.2, 3, 3, 3, 3, false, 0, 0, 2);
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 2;
                }
            };
        }
        public Action strafeLeft() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        encoderDrive(0.2, 2, 2, 2, 2, true, 0, 0, 2);
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 2;
                }
            };
        }
    }

    private double done = 0;
    private double position = 0;

    @Override
    public void runOpMode() {
        // initial pose and RR drive creation (preserve original)
        Pose2d initialPose = new Pose2d(-50, 47, Math.toRadians(14.9));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // initialize vision processor (will be used during INIT loop)
        initAprilTag();

        // hardware init (clean and avoid duplication)
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

        // motor directions (fixed earlier typo and kept consistent)
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

        // display starting encoders
        telemetry.addData("Starting at", "%7d : %7d : %7d : %7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        telemetry.update();

        // set initial servo positions
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);

        // -------------------------
        // APRILTAG INIT LOOP (runs during INIT until play pressed)
        // -------------------------
        telemetry.addLine("Scanning for AprilTag during INIT...");
        telemetry.update();

        ElapsedTime initScanTimer = new ElapsedTime();
        initScanTimer.reset();

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

            // small sleep to reduce CPU usage
            sleep(50);
        }

        // WAIT FOR MATCH START
        waitForStart();

        // Immediately disable/close the webcam to free resources
        if (allSeeingEye != null) {
            try {
                allSeeingEye.close();
            } catch (Exception e) {
                // ignore close errors; continue
            }
            allSeeingEye = null;
        }

        // initialize PID controllers AFTER start
        pid1 = new PIDControl(shooter1, SHOOTER_TICKS_PER_REV);
        pid2 = new PIDControl(shooter2, SHOOTER_TICKS_PER_REV);
        pid3 = new PIDControl(shooter3, SHOOTER_TICKS_PER_REV);

        // start shooter background thread (runs for 30 seconds)
        shooterTimer.reset();
        Thread shooterThread = new Thread(() -> {
            while (opModeIsActive() && shooterTimer.seconds() < 30.0) {
                try {
                    double out1 = pid1.update(targetRPM);
                    double out2 = pid2.update(targetRPM);
                    double out3 = pid3.update(targetRPM);

                    shooter1.setPower(out1);
                    shooter2.setPower(out2);
                    shooter3.setPower(out3);

                    Thread.sleep(20); // PID sampling interval
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    // If something goes wrong, stop motors and break
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
                .strafeTo(new Vector2d(-12, 20))
                .turn(Math.toRadians(-44.1))
                .waitSeconds(3);
        Pose2d newPose = new Pose2d(-12, 20, Math.toRadians(-44.1));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(newPose)
                .lineToY(56)
                .waitSeconds(3);
        Pose2d new2Pose = new Pose2d(-40, 56, Math.toRadians(-95.2059));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new2Pose)
                .strafeTo(new Vector2d(-50, 47))
                .turn(44.1)
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-16, 38))
                .build();

        // choose trajectories (kept identical flow)
        position = 1;
        Action trajectoryActionChosen;
        if (position == 1) trajectoryActionChosen = tab1.build();
        else if (position == 2) trajectoryActionChosen = tab2.build();
        else trajectoryActionChosen = tab3.build();

        Shooter shoot = new Shooter(hardwareMap);

        // We already scanned in INIT, so remove runtime scan. Continue the route:
        Actions.runBlocking(new SequentialAction(shoot.Fire(), trajectoryActionChosen));

        position = 2;
        if (position == 1) trajectoryActionChosen = tab1.build();
        else if (position == 2) trajectoryActionChosen = tab2.build();
        else trajectoryActionChosen = tab3.build();
        Actions.runBlocking(new SequentialAction(shoot.intake(), trajectoryActionChosen));

        position = 3;
        if (position == 1) trajectoryActionChosen = tab1.build();
        else if (position == 2) trajectoryActionChosen = tab2.build();
        else trajectoryActionChosen = tab3.build();
        Actions.runBlocking(new SequentialAction(shoot.Fire(), trajectoryActionChosen));

        Actions.runBlocking(new SequentialAction(trajectoryActionCloseOut));

        // BEFORE firing, wait a short timeout for shooter RPMs to come up and only fire if within tolerance
        double waitStart = getRuntime();
        double waitTimeout = 5.0; // seconds to wait for reaching speed
        boolean allAtTarget = false;
        while (opModeIsActive() && (getRuntime() - waitStart) < waitTimeout) {
            double r1 = pid1.getLastRPM();
            double r2 = pid2.getLastRPM();
            double r3 = pid3.getLastRPM();
            telemetry.addData("RPMs", "%4.0f, %4.0f, %4.0f", r1, r2, r3);
            telemetry.update();
            if (r1 >= targetRPM - 60 && r2 >= targetRPM - 60 && r3 >= targetRPM - 60) {
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

        // reset servos to safe positions (preserve original)
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }

    // Initialize VisionPortal + AprilTagProcessor
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
