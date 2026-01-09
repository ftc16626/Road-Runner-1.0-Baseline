package org.firstinspires.ftc.teamcode.TeleOp;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "willwork", group = "robot")
@Disabled
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
    public static double Kp1 = 0.95;
    public static double Ki1 = 0;
    public static double Kd1 = 0.1;
    public static double Kp2 = 0.87;
    public static double Ki2 = 0;
    public static double Kd2 = 0.1;
    public static double Kp3 = 0.89;
    public static double Ki3 = 0;
    public static double Kd3 = 0.1;
    // ----- PID states for each motor -----
    PIDtest.PIDState1 pid1 = new PIDtest.PIDState1();
    PIDtest.PIDState2 pid2 = new PIDtest.PIDState2();
    PIDtest.PIDState3 pid3 = new PIDtest.PIDState3();

    public static double targetVelocity = 1250;


    // Constants / tuning
    private static final double TICKS_PER_REV = 28.0;
    private double targetRPM = 1350.0;             // default
    private double hoodPositionClose = 0.225;      // safe hood positions
    private double hoodPositionFar = 0.40;         // do not exceed hardware limits
    private final ElapsedTime loopTimer = new ElapsedTime();
    private  double hoodPos;

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
        pid1.timer.reset();
        pid2.timer.reset();
        pid3.timer.reset();
        rightHoodServo.setPosition(0);
        leftHoodServo.setPosition(0);
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
            if (gamepad2.dpad_left){
                targetVelocity = 1250;
                hoodPos = 0.225;
                rightHoodServo.setPosition(hoodPos);
                leftHoodServo.setPosition(hoodPos);
            } else if (gamepad2.dpad_right){
                targetVelocity = 2500;
                hoodPos = 0.4;
                rightHoodServo.setPosition(hoodPos);
                leftHoodServo.setPosition(hoodPos);
            }else if (gamepad2.dpad_up) {
                targetVelocity = 1050;
                hoodPos = 0.025;
                rightHoodServo.setPosition(hoodPos);
                leftHoodServo.setPosition(hoodPos);
            }

            // --- Shooter control ---
            if (gamepad2.right_bumper) {
                // update independent PIDs -- pass gamepad2 for rumble
                double v1 = shooter1.getVelocity();
                double v2 = shooter2.getVelocity();
                double v3 = shooter3.getVelocity();

                // ---- Compute PID ----
                double power1 = PID1(targetVelocity, v1, pid1);
                double power2 = PID2(targetVelocity, v2, pid2);
                double power3 = PID3(targetVelocity, v3, pid3);

                // ---- Apply power ----
                shooter1.setPower(power1);
                shooter2.setPower(power2);
                shooter3.setPower(power3);


            } else if (gamepad2.left_bumper) {
                // reverse to un-jam
                setAllShooterPower(-0.25);
                // keep flippers in a position to clear jam
                flipper1.setPosition(0.6);
                flipper2.setPosition(0.6);
                flipper3.setPosition(0.41);

                // reset PID internal integrators so we don't wind up


            } else {
                // idle shooters
                setAllShooterPower(0.0);

            }

            // --- Intake / roller (gamepad1) ---
            if (gamepad1.right_bumper) {
                rollerServo.setPower(1.0);
            } else if (gamepad1.left_bumper) {
                rollerServo.setPower(-1.0);
            } else {
                rollerServo.setPower(0.0);
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
               // telemetry.addData("RPM1", "%d", (int) pid1.;
               // telemetry.addData("RPM2", "%d", (int) pid2.getRPM());
                //telemetry.addData("RPM3", "%d", (int) pid3.getRPM());
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

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);


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
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // servos and actuators
        rollerServo = hardwareMap.get(CRServo.class, "roller");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        leftHoodServo.setDirection(Servo.Direction.REVERSE);
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        rightHoodServo.setDirection(Servo.Direction.FORWARD);
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
        //pid1 = new PIDState2(shooter1);
      //  pid2 = new PIDState1(shooter2);
       // pid3 = new PIDState1(shooter3);

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
    public double PID1(double reference1, double state1, PIDtest.PIDState1 pid) {

        double error1 = reference1 - state1;

        double dt = pid.timer.seconds();
        pid.timer.reset();

        if (dt == 0) dt = 0.001; // prevent div by zero

        pid.integral1 += error1 * dt;
        double derivative1 = (error1 - pid.lastError1) / dt;

        pid.lastError1 = error1;
        double output1 = (Kp1 * error1) + (Ki1 * pid.integral1) + (Kd1 * derivative1);

// Clamp to motor power limits
        output1 = Math.max(-1, Math.min(1, output1));
        return output1;
    }

    // ----- PID State Class -----
    static class PIDState1 {
        public double lastError1 = 0;
        public double integral1 = 0;
        public ElapsedTime timer = new ElapsedTime();
    }
    public double PID2(double reference2, double state2, PIDtest.PIDState2 pid) {

        double error2 = reference2 - state2;

        double dt = pid.timer.seconds();
        pid.timer.reset();

        if (dt == 0) dt = 0.001; // prevent div by zero

        pid.integral2 += error2 * dt;
        double derivative2 = (error2 - pid.lastError2) / dt;

        pid.lastError2 = error2;

        double output2 = (Kp2 * error2) + (Ki2 * pid.integral2) + (Kd2 * derivative2);

// Clamp to motor power limits
        output2 = Math.max(-1, Math.min(1, output2));
        return output2;
    }

    // ----- PID State Class -----
    static class PIDState2 {
        public double lastError2 = 0;
        public double integral2 = 0;
        public ElapsedTime timer = new ElapsedTime();
    }
    public double PID3(double reference3, double state3, PIDtest.PIDState3 pid) {

        double error3 = reference3 - state3;

        double dt = pid.timer.seconds();
        pid.timer.reset();

        if (dt == 0) dt = 0.001; // prevent div by zero

        pid.integral3 += error3 * dt;
        double derivative3 = (error3 - pid.lastError3) / dt;

        pid.lastError3 = error3;

        double output3 = (Kp3 * error3) + (Ki3 * pid.integral3) + (Kd3 * derivative3);

// Clamp to motor power limits
        output3 = Math.max(-1, Math.min(1, output3));
        return output3;
    }

    // ----- PID State Class -----
    static class PIDState3 {
        public double lastError3 = 0;
        public double integral3 = 0;
        public ElapsedTime timer = new ElapsedTime();
    }
}