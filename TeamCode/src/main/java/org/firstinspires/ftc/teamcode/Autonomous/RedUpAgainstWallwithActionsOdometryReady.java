package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import org.firstinspires.ftc.teamcode.ComponentSubClasses.DriveSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.HoodSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.IntakeSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.ShooterSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.VisionSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.ColorSensorSubsystem;

@Autonomous(name="RedUpAgainstWallwithActionsOdometryReady", group="Robot")
@Disabled
public class RedUpAgainstWallwithActionsOdometryReady extends LinearOpMode {

    // Subsystems
    private DriveSubsystemOdometryReady drive;
    private HoodSubsystemOdometryReady hood;
    private IntakeSubsystemOdometryReady intake;
    private ShooterSubsystemOdometryReady shooter;
    private VisionSubsystemOdometryReady vision;
    private ColorSensorSubsystem colorSensors;

    // Hardware cached for legacy encoderDrive
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private CRServo intakeServo;
    private Servo servoI, servoII, servoIII;
    private Servo leftHoodServo, rightHoodServo;
    private VisionPortal allSeeingEye;
    private AprilTagProcessor aprilTag;

    // Motion constants
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // shooter constants
    private static final double TARGET_RPM = 2300.0;
    private static final double SHOOTER_RUN_SECONDS = 30.0;

    // misc
    private ElapsedTime runtime = new ElapsedTime();
    private double artifactPattern = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize subsystems
        drive = new DriveSubsystemOdometryReady(hardwareMap);
        hood = new HoodSubsystemOdometryReady(hardwareMap);
        intake = new IntakeSubsystemOdometryReady(hardwareMap);
        shooter = new ShooterSubsystemOdometryReady(hardwareMap);
        vision = new VisionSubsystemOdometryReady(hardwareMap);

        colorSensors = new ColorSensorSubsystem(hardwareMap,
                "first", "second", "third", "fourth", "fifth", "sixth");

        // Cache hardware for legacy calls
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBMotor");

        intakeServo = hardwareMap.get(CRServo.class, "roller");

        servoI = hardwareMap.get(Servo.class, "flipper1");
        servoII = hardwareMap.get(Servo.class, "flipper2");
        servoIII = hardwareMap.get(Servo.class, "flipper3");

        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");

        // Initialize VisionSubsystem / AprilTag
        
        aprilTag = vision.getAprilTagProcessor();
        allSeeingEye = vision.getVisionPortal();

        // Motor directions & modes
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // Color sensor lights and gain via subsystem
        colorSensors.enableLights(true);
        colorSensors.setGain(1.0);

        // Default flipper positions
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);

        leftHoodServo.setPosition(0.0);
        rightHoodServo.setPosition(0.0);

        telemetry.addLine("Init complete - waiting for start");
        telemetry.addData("Color sensors found", colorSensors.getFoundSensorCount());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        ShooterActions shoot = new ShooterActions();

        // Scan AprilTag (blocking)
        Actions.runBlocking(new SequentialAction(shoot.Scan()));

        // Strafe left + turn + shooter spin and fire
        Actions.runBlocking(new ParallelAction(
                new SequentialAction(shoot.strafeLeft(), shoot.turn()),
                shoot.shooterSpinAndFire()
        ));

        // Drive sequence (encoderDrive)
        encoderDrive(0.25, -8, -8, -8, -8, false, 0, 0, 3);
        encoderDrive(0.25, 5, -5, 5, -5, false, 0, 0, 3);
        encoderDrive(0.25, -5, -5, -5, -5, true, 0, 0, 3);
        encoderDrive(0.25, 1.5, 1.5, 1.5, 1.5, false, 0, 0, 2);
        encoderDrive(0.25, 11, 11, 11, 11, false, -1, 0, 3);
        encoderDrive(0.05, 10, 10, 10, 10, true, -1, 0, 3);

        // Final turn + ensure shooter firing
        Actions.runBlocking(new ParallelAction(
                shoot.turn2(),
                shoot.shooterSpinAndFire()
        ));

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // -----------------------------
    // EncoderDrive function
    // -----------------------------
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             boolean strafe, double intakePower, double hoodAngle,
                             double timeoutS) {

        if (!opModeIsActive()) return;

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
                (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() &&
                        leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            // hood angle & intake power
            leftHoodServo.setPosition(hoodAngle);
            rightHoodServo.setPosition(hoodAngle);
            intakeServo.setPower(intakePower);

            // Optional: color debug telemetry
            String[] colors = colorSensors.readAllColorNames();
            telemetry.addData("Colors", String.join(",", colors));
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);
    }

    // -----------------------------
    // ShooterActions inner class
    // -----------------------------
    private class ShooterActions {
        private ElapsedTime timer = new ElapsedTime();
        private ElapsedTime servoTimer = new ElapsedTime();
        private ElapsedTime totalTimer = new ElapsedTime();

        public Action Scan() {
            return new Action() {
                private boolean initialized = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        List<AprilTagDetection> detections = vision.getAprilTagProcessor().getDetections();
                        if (!detections.isEmpty()) {
                            artifactPattern = detections.get(0).id;
                            // remap as original
                            if (artifactPattern == 23) artifactPattern = 21;
                            else if (artifactPattern == 21) artifactPattern = 22;
                            else if (artifactPattern == 22) artifactPattern = 23;
                        }
                        initialized = true;
                        timer.reset();
                    }
                    return timer.seconds() < 2;
                }
            };
        }

        public Action strafeLeft() {
            return timedDriveAction(() -> encoderDrive(0.2, 2, 2, 2, 2, true, 0, 0, 2), 2);
        }

        public Action turn() {
            return timedDriveAction(() -> {
                encoderDrive(0.25, 9.5, -9.5, 9.5, -9.5, false, 0, 0, 1);
                encoderDrive(0.25, 2, 2, 2, 2, false, 0, 0, 1);
            }, 1);
        }

        public Action turn2() {
            return timedDriveAction(() -> {
                encoderDrive(0.25, 7, 7, 7, 7, true, 0, 0, 2);
            }, 2);
        }

        public Action shooterSpinAndFire() {
            return new Action() {
                private boolean initialized = false;
                private boolean fired = false;
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        shooter.resetSamplers();
                        servoTimer.reset();
                        totalTimer.reset();
                        initialized = true;
                    }

                    // Maintain shooter PID
                    shooter.pid1.update(TARGET_RPM, gamepad1);
                    shooter.pid2.update(TARGET_RPM, gamepad1);
                    shooter.pid3.update(TARGET_RPM, gamepad1);

                    // Only fire once
                    if (!fired &&
                            Math.abs(shooter.pid1.getRPM() - TARGET_RPM) < 60 &&
                            Math.abs(shooter.pid2.getRPM() - TARGET_RPM) < 60 &&
                            Math.abs(shooter.pid3.getRPM() - TARGET_RPM) < 60) {
                        fired = true;
                        servoTimer.reset();
                        // simplified firing logic (you can expand per pattern if needed)
                        servoI.setPosition(0.9);
                        servoII.setPosition(0.9);
                        servoIII.setPosition(0.53);
                    }

                    return totalTimer.seconds() < SHOOTER_RUN_SECONDS;
                }
            };
        }

        private Action timedDriveAction(final Runnable r, final double seconds) {
            return new Action() {
                private boolean initialized = false;
                private ElapsedTime t = new ElapsedTime();
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) { r.run(); initialized = true; t.reset(); }
                    return t.seconds() < seconds;
                }
            };
        }
    }
}
