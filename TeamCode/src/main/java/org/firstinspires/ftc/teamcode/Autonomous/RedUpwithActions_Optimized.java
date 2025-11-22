package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "TheRedUpAutoYouShouldUse_Optimized", group = "Robot")
public class RedUpwithActions_Optimized extends LinearOpMode {

    // Drive
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Shooter
    private DcMotorEx shooter;
    private double targetRPM = 2300.0;
    private double ticksPerRevolution = 28.0; // if using velocity in ticks/sec
    private double targetVelocity; // in ticks/sec

    // Servos
    private Servo servoI, servoII, servoIII;
    private Servo leftHood, rightHood;
    private CRServo intake;

    // Color sensors (kept but not polled every loop)
    private NormalizedColorSensor color1, color2, color3, color4, color5, color6;

    // AprilTag
    private VisionPortal camera;
    private AprilTagProcessor aprilTag;
    private double artifactPattern = -1;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoStateTimer = new ElapsedTime();

    // Simple non-blocking shooter fire state machine
    private enum FireState { IDLE, PREP, FIRE_STEP1, FIRE_STEP2, DONE }
    private FireState fireState = FireState.IDLE;

    // Constants
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        initHardware(hardwareMap);
        initAprilTag();

        // compute velocity target in ticks/sec if using setVelocity
        // targetRPM -> revolutions per minute; convert to ticks per second
        targetVelocity = (targetRPM / 60.0) * ticksPerRevolution;

        // initial positions
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);
        leftHood.setPosition(0.0);
        rightHood.setPosition(0.0);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        runtime.reset();

        // 1) Quick one-frame AprilTag scan (very short)
        doQuickAprilScan();

        // 2) Move into scan area (faster than original 0.2)
        encoderDrive(0.4, -2, -2, -2, -2, 1.0);
        encoderDrive(0.4, -5, -5, -5, -5, 1.2);
        encoderDrive(0.4, -5, -5, -5, -5, 1.2);

        // 3) Spin up shooter using motor's velocity controller (fast)
        spinUpShooterBlocking(0.5); // short blocking spin-up

        // 4) Move into shooting position faster
        encoderDrive(0.4, 2, -2, 2, -2, 0.6);
        encoderDrive(0.4, 1, 1, 1, 1, 0.4);

        // 5) Fire sequence (non-blocking state machine executed in short loop)
        runFireSequenceBlocking(1.2); // runs the non-blocking state machine but returns when done

        // 6) Exit shooting area
        encoderDrive(0.5, 4, 4, 4, 4, 1.0);

        // Final
        shooter.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void initHardware(HardwareMap hw) {
        leftFront = hw.get(DcMotor.class, "LFMotor");
        rightFront = hw.get(DcMotor.class, "RFMotor");
        leftBack = hw.get(DcMotor.class, "LBMotor");
        rightBack = hw.get(DcMotor.class, "RBMotor");

        shooter = hw.get(DcMotorEx.class, "shooter");

        intake = hw.get(CRServo.class, "roller");
        leftHood = hw.get(Servo.class, "leftHoodServo");
        rightHood = hw.get(Servo.class, "rightHoodServo");

        servoI = hw.get(Servo.class, "flipper1");
        servoII = hw.get(Servo.class, "flipper2");
        servoIII = hw.get(Servo.class, "flipper3");

        color1 = hw.get(NormalizedColorSensor.class, "first");
        color2 = hw.get(NormalizedColorSensor.class, "second");
        color3 = hw.get(NormalizedColorSensor.class, "third");
        color4 = hw.get(NormalizedColorSensor.class, "fourth");
        color5 = hw.get(NormalizedColorSensor.class, "fifth");
        color6 = hw.get(NormalizedColorSensor.class, "sixth");

        // Directions: set once
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Enable lights on color sensors if available
        tryEnableLight(color1);
        tryEnableLight(color2);
        tryEnableLight(color3);
        tryEnableLight(color4);
        tryEnableLight(color5);
        tryEnableLight(color6);
    }

    private void tryEnableLight(NormalizedColorSensor s) {
        if (s instanceof SwitchableLight) ((SwitchableLight) s).enableLight(true);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(false)
                .setDrawTagOutline(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .build();

        camera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "allSeeingEye"))
                .addProcessor(aprilTag)
                .build();
    }

    // One-frame quick scan that closes camera immediately
    private void doQuickAprilScan() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection d : detections) {
                if (d.id == 21 || d.id == 22 || d.id == 23) {
                    artifactPattern = d.id;
                    telemetry.addData("Tag", d.id);
                }
            }
        } else {
            telemetry.addData("AprilTag", "Not seen");
        }
        telemetry.update();
        camera.close();
    }

    // Simplified encoder drive that sets RUN_TO_POSITION and waits with small telemetry overhead
    private void encoderDrive(double speed, double lfIn, double rfIn, double lbIn, double rbIn, double timeoutS) {
        if (!opModeIsActive()) return;

        int newLF = leftFront.getCurrentPosition() + (int) (lfIn * COUNTS_PER_INCH);
        int newRF = rightFront.getCurrentPosition() + (int) (rfIn * COUNTS_PER_INCH);
        int newLB = leftBack.getCurrentPosition() + (int) (lbIn * COUNTS_PER_INCH);
        int newRB = rightBack.getCurrentPosition() + (int) (rbIn * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newLF);
        rightFront.setTargetPosition(newRF);
        leftBack.setTargetPosition(newLB);
        rightBack.setTargetPosition(newRB);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Use absolute power; sign will handle direction already set
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        runtime.reset();
        // Wait loop but keep it tight and lightweight
        while (opModeIsActive() && runtime.seconds() < timeoutS &&
                (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
            // minimal telemetry to save time; update only occasionally
            if ((int) (runtime.milliseconds()) % 200 < 20) {
                telemetry.addData("LF pos", leftFront.getCurrentPosition());
                telemetry.addData("Tag", artifactPattern);
                telemetry.update();
            }
            // Allow other system tasks
            idle();
        }

        // stop
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // restore modes
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // small non-blocking pause
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() < 50) idle();
    }

    // Use motor controller's velocity where available; short blocking spin-up
    private void spinUpShooterBlocking(double maxWaitSeconds) {
        // try to use setVelocity API if available. If not, fallback to open-loop power
        try {
            shooter.setVelocity(targetVelocity);
        } catch (Exception e) {
            shooter.setPower(0.9);
        }

        // wait a small amount but not long
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < Math.min(0.6, maxWaitSeconds)) {
            idle();
        }
    }

    // Blocking wrapper around a non-blocking fire state machine
    private void runFireSequenceBlocking(double maxSeconds) {
        fireState = FireState.PREP;
        servoStateTimer.reset();
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive() && loopTimer.seconds() < maxSeconds && fireState != FireState.DONE) {
            // keep shooter running
            idle();
            switch (fireState) {
                case PREP:
                    // set middle servo to load position quickly then move on
                    servoII.setPosition(0.9);
                    servoStateTimer.reset();
                    fireState = FireState.FIRE_STEP1;
                    break;

                case FIRE_STEP1:
                    if (servoStateTimer.milliseconds() < 150) {
                        servoI.setPosition(0.47);
                    } else {
                        servoI.setPosition(0.9);
                        servoStateTimer.reset();
                        fireState = FireState.FIRE_STEP2;
                    }
                    break;

                case FIRE_STEP2:
                    if (servoStateTimer.milliseconds() < 250) {
                        servoIII.setPosition(0.53);
                    } else {
                        // reset to safe positions
                        servoIII.setPosition(0.1);
                        servoII.setPosition(0.5);
                        servoI.setPosition(0.5);
                        fireState = FireState.DONE;
                    }
                    break;

                default:
                    fireState = FireState.DONE;
                    break;
            }
        }

        // ensure final positions
        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);
    }
}

