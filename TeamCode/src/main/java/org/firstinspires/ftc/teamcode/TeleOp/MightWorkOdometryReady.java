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
import org.firstinspires.ftc.teamcode.ComponentSubClasses.DriveSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.HoodSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.IntakeSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.ShooterSubsystemOdometryReady;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.VisionSubsystemOdometryReady;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Main TeleOp class, odometry-ready.
 * Uses subsystems for drive, shooter, hood, intake, and vision.
 * All current PID, servo, and button mappings preserved.
 * (Is this language simple enough?)
 */
@TeleOp(name = "mightworkOdometryReady", group = "robot")

public class MightWorkOdometryReady extends LinearOpMode {

    // -------------------- Subsystems --------------------
    private DriveSubsystemOdometryReady drive;
    private ShooterSubsystemOdometryReady shooter;
    private HoodSubsystemOdometryReady hood;
    private IntakeSubsystemOdometryReady intake;
    private VisionSubsystemOdometryReady vision;

    // Telemetry and loop control
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL = 0.20;

    // Shooter constants
    private double curTargetVelocity = 1213.333333;
    private double hoodPosClose = 0.225;
    private double hoodPosFar = 0.40;
    private double hoodPosGoal = 0.025;

    // Layout reference for TeleOp (optional)
    private View relativeLayout;
    public Servo leftHoodServo, rightHoodServo;
    public Servo flipper1, flipper2, flipper3;

    // -------------------- Odometry placeholders --------------------
    // private int leftOdomPrev = 0;
    // private int rightOdomPrev = 0;
    // private int frontOdomPrev = 0;
    // public double currentX = 0.0;    // in inches or meters
    // public double currentY = 0.0;
    // public double heading = 0.0;     // in radians
    private void setHood(double pos) {
        leftHoodServo.setPosition(pos);
        rightHoodServo.setPosition(pos);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        // -------------------- Initialize subsystems --------------------
        drive = new DriveSubsystemOdometryReady(hardwareMap);
        shooter = new ShooterSubsystemOdometryReady(hardwareMap);
        hood = new HoodSubsystemOdometryReady(hardwareMap);
        intake = new IntakeSubsystemOdometryReady(hardwareMap);
        vision = new VisionSubsystemOdometryReady(hardwareMap);
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHoodServo.setDirection(Servo.Direction.REVERSE);
        rightHoodServo.setDirection(Servo.Direction.FORWARD);
        flipper1 = hardwareMap.get(Servo.class,"flipper1");
        flipper2 = hardwareMap.get(Servo.class,"flipper2");
        flipper3 = hardwareMap.get(Servo.class,"flipper3");
        // Optional layout
        int relativeLayoutId = hardwareMap.appContext.getResources()
                .getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        try {
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        } catch (Exception ignored) {}


        telemetry.addData("Status", "Ready - press START");
        telemetry.update();
        waitForStart();


        loopTimer.reset();
        telemetryTimer.reset();

        // -------------------- Main loop --------------------
        while (opModeIsActive()) {
            double dtLoop = loopTimer.seconds();
            loopTimer.reset();

            // -------------------- Odometry update placeholder --------------------
            /*
            int leftOdomPos = drive.leftOdom.getCurrentPosition();
            int rightOdomPos = drive.rightOdom.getCurrentPosition();
            int frontOdomPos = drive.frontOdom.getCurrentPosition();
            int leftDelta = leftOdomPos - leftOdomPrev;
            int rightDelta = rightOdomPos - rightOdomPrev;
            int frontDelta = frontOdomPos - frontOdomPrev;
            double leftInches = drive.ticksToInches(leftDelta);
            double rightInches = drive.ticksToInches(rightDelta);
            double frontInches = drive.ticksToInches(frontDelta);
            // TODO: Update currentX, currentY, heading here
            leftOdomPrev = leftOdomPos;
            rightOdomPrev = rightOdomPos;
            frontOdomPrev = frontOdomPos;
            */

            // -------------------- Drive --------------------
            drive.driveMecanum(gamepad1);


            // -------------------- Intake --------------------
            intake.controlIntake(gamepad1);

            // -------------------- Hood / Flippers --------------------
            hood.controlFlippers(gamepad2);
            if (gamepad2.dpad_left){
                curTargetVelocity = 1313.333333;
                setHood(hoodPosClose);
            } else if (gamepad2.dpad_right){
                curTargetVelocity = 1560;
                setHood(hoodPosFar);
            }
            if (gamepad2.left_bumper){
                flipper1.setPosition(0.6);
                flipper2.setPosition(0.6);
                flipper3.setPosition(0.41);
            }
            // -------------------- Shooter --------------------
            shooter.controlShooter(curTargetVelocity, gamepad2);

            // -------------------- Vision / AprilTag --------------------
            if (gamepad1.square && vision.getAprilTagProcessor() != null) {
                List<AprilTagDetection> dets = vision.getAprilTagProcessor().getDetections();
                if (!dets.isEmpty()) {
                    for (AprilTagDetection d : dets) {
                        if (d.metadata != null) {
                            telemetry.addData("TagID", d.id);
                            telemetry.addData("Pose", "%.2f, %.2f, %.2f",
                                    d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
                        }
                    }
                } else {
                    telemetry.addData("AprilTag", "Not detected");
                }
            }

            // -------------------- Telemetry --------------------
            if (telemetryTimer.seconds() >= TELEMETRY_INTERVAL) {
                telemetryTimer.reset();

                telemetry.update();
            }

            sleep(15);
        }
    }
}
