package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensors;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;
import java.util.List;

@Autonomous(name="BLUE_UP_AGAINST_THE_GOAL_MEET4", group="Robot")
public class blueUpAgainstGoalMeet4 extends LinearOpMode {

    // -------------------- Hardware --------------------
    private DcMotorEx shooter1, shooter2, shooter3;
    // private Servo servoI, servoII, servoIII;
    private Servo leftHood, rightHood;
    //  private CRServo Intake;

    //  private NormalizedColorSensor colorSensorI, colorSensorII;
    //  private NormalizedColorSensor colorSensorIII, colorSensorIV;
    //  private NormalizedColorSensor colorSensorV, colorSensorVI;
    private VisionPortal allSeeingEye;
    private AprilTagProcessor aprilTag;

    private double curTargetVelocity = 1253.33333-150;
    private ElapsedTime shooterTimer = new ElapsedTime();

    private double hoodPosClose = 0.225;
    private double targetRPM = 2050;
    private CRServo intake;

    private int artifactPattern = 0;
    private enum FireState {
        WAIT_FOR_COLOR,
        SERVO_OUT,
        SERVO_BACK,
        DONE
    }

    public class Everything {
        private CRServo intake;
        private Servo servoI, servoII, servoIII;
        private NormalizedColorSensor colorSensorI, colorSensorII;
        private NormalizedColorSensor colorSensorIII, colorSensorIV;
        private NormalizedColorSensor colorSensorV, colorSensorVI;

        ElapsedTime timer;

        private FireState lastState = null;


        public Everything(HardwareMap hardwareMapmap) {


            intake = hardwareMap.get(CRServo.class, "roller");

            servoI = hardwareMap.get(Servo.class, "flipper1");
            servoII = hardwareMap.get(Servo.class, "flipper2");
            servoIII = hardwareMap.get(Servo.class, "flipper3");

            colorSensorI = hardwareMap.get(NormalizedColorSensor.class, "first");
            colorSensorII = hardwareMap.get(NormalizedColorSensor.class, "second");
            colorSensorIII = hardwareMap.get(NormalizedColorSensor.class, "third");
            colorSensorIV = hardwareMap.get(NormalizedColorSensor.class, "fourth");
            colorSensorV = hardwareMap.get(NormalizedColorSensor.class, "fifth");
            colorSensorVI = hardwareMap.get(NormalizedColorSensor.class, "sixth");

            enableColorSensorLight(colorSensorI);
            enableColorSensorLight(colorSensorII);
            enableColorSensorLight(colorSensorIII);
            enableColorSensorLight(colorSensorIV);
            enableColorSensorLight(colorSensorV);
            enableColorSensorLight(colorSensorVI);
        }

        private ArtifactColor detectColor(NormalizedColorSensor s1) {
            NormalizedRGBA c1 = s1.getNormalizedColors();

            float normRed, normGreen, normBlue;
            normRed = c1.red;
            normGreen = c1.green;
            normBlue = c1.blue;

            // robust classifier

            if (normRed <0.045 && normGreen >0.06 && normBlue >0.04) {
                return ArtifactColor.GREEN;
            } else if (normRed <0.08 && normGreen <0.1 && normBlue >0.06) {
                return ArtifactColor.PURPLE;
            }
            else return ArtifactColor.NONE;
        }

        public Action hood() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        leftHood.setPosition(hoodPosClose);
                        rightHood.setPosition(hoodPosClose);
                        curTargetVelocity = 1253.33333;
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 0.5;
                }
            };
        }
        public Action roller() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        intake.setPower(-1);
                        timer = new ElapsedTime();
                        initialized = true;
                    }

                    if (timer.seconds() >= 2.2) {
                        intake.setPower(0);   // STOP THE SERVO
                        return false;         // Action done
                    }

                    return true;
                }
            };
        }

        public Action fire() {
            return new Action() {

                private FireState state = FireState.WAIT_FOR_COLOR;
                private static final double FIRE_TIMEOUT_SEC = 3.0;
                private final ElapsedTime timer = new ElapsedTime();


                private ArtifactColor[] pattern = mapPattern(artifactPattern);
                private int index = 0;

                private boolean initialized = false;
                private boolean shotThisStep = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (state != lastState) {
                        timer.reset();
                        lastState = state;
                    }

                    // One-time init
                    if (!initialized) {
                        timer.reset();
                        initialized = true;
                    }

                    // Finish condition (Action COMPLETE)
                    if (index >= pattern.length) {
                        packet.put("Fire", "DONE");
                        return false;
                    }

                    // Read sensors every loop
                    ArtifactColor s1 = detectColor(colorSensorI);
                    ArtifactColor s2 = detectColor(colorSensorII);
                    ArtifactColor s3 = detectColor(colorSensorIII);
                    ArtifactColor s4 = detectColor(colorSensorIV);
                    ArtifactColor s5 = detectColor(colorSensorV);
                    ArtifactColor s6 = detectColor(colorSensorVI);

                    // Telemetry (THIS IS GOLD)
                    telemetry.addData("Fire State", state);
                    telemetry.addData("Pattern Index", index);
                    telemetry.addData("Target", pattern[index]);
                    telemetry.addData("S1", s1);
                    telemetry.addData("S2", s2);
                    telemetry.addData("S3", s3);
                    telemetry.addData("Timer (ms)", timer.milliseconds());
                    telemetry.update();

                    switch (state) {


                        // Accept match from ANY sensor
                        case WAIT_FOR_COLOR:

                            // NORMAL FIRE CONDITION
                            if (!shotThisStep &&
                                    (s1 == pattern[index] ||
                                            s2 == pattern[index] ||
                                            s3 == pattern[index] ||
                                            s4 == pattern[index] ||
                                            s5 == pattern[index] ||
                                            s6 == pattern[index])) {

                                if (s1 == pattern[index] || s2 == pattern[index]) servoIII.setPosition(0.1);
                                else if (s3 == pattern[index] || s4 == pattern[index]) servoII.setPosition(0.9);
                                else servoI.setPosition(0.9);

                                shotThisStep = true;
                                state = FireState.SERVO_OUT;
                            }

                            // FAILSAFE NO SHOT AFTER 3 SECONDS
                            else if (timer.seconds() > FIRE_TIMEOUT_SEC) {

                                telemetry.addLine(" FIRE TIMEOUT — SKIPPING SHOT");
                                telemetry.update();

                                // Reset servos to safe position
                                servoI.setPosition(0.47);
                                servoII.setPosition(0.47);
                                servoIII.setPosition(0.55);

                                // Skip this target and move on
                                index++;
                                shotThisStep = false;
                                state = FireState.WAIT_FOR_COLOR;
                            }
                            break;

                    }

                    return true; // keep Action alive
                }
            };
        }
    }


    // -------------------- Artifact Color Enum --------------------
    private enum ArtifactColor { GREEN, PURPLE, NONE }


    // -------------------- Autonomous Logic --------------------
    @Override
    public void runOpMode() throws InterruptedException {


        // -------------------- Initialize Drive --------------------


        Pose2d startPose = (new Pose2d(-52, -47,Math.toRadians(-126)));
        Pose2d pickup1 = (new Pose2d(-8, -25, Math.toRadians(-85)));
        Pose2d pickup1end = (new Pose2d(-8, -50, Math.toRadians(-85)));
        Pose2d moveto2 = (new Pose2d(-10, -10, Math.toRadians(-138)));
        Pose2d pickup2 = (new Pose2d(16, -26, Math.toRadians(-82)));
        Pose2d pickup2end = (new Pose2d(20, -60, Math.toRadians(-82)));
        Pose2d pickup3 = (new Pose2d(38.5, -20, Math.toRadians(-82)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action driveAction = drive.actionBuilder(pickup1)
                .setReversed(false)
                .lineToY(-50)
                .build();
        Action driveAction2 = drive.actionBuilder(pickup2)
                .setReversed(false)
                .lineToY(-52)
                .build();
        Action driveAction3 = drive.actionBuilder(pickup3)
                .setReversed(false)
                .lineToY(-60)
                .build();


        // -------------------- Initialize Hardware --------------------
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");
        intake = hardwareMap.get(CRServo.class, "roller");
        Everything Everything = new Everything(hardwareMap);



        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter3.setDirection(DcMotorEx.Direction.FORWARD);
        double F1 = 13.9;
        double P1 = 80;


        double F2 = 12.4;
        double P2 = 60;


        double F3 = 15.45;
        double P3 = 60;


        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P2,0,0,F2);
        PIDFCoefficients pidfCoefficients3 = new PIDFCoefficients(P3,0,0,F3);


        //   servoI = hardwareMap.get(Servo.class, "flipper3");
        //   servoII = hardwareMap.get(Servo.class, "flipper2");
        //  servoIII = hardwareMap.get(Servo.class, "flipper1");
        // Intake = hardwareMap.get(Servo.class, "roller");

        leftHood = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHood = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHood.setDirection(Servo.Direction.REVERSE);
        leftHood.setPosition(0.0);
        rightHood.setPosition(0.0);



        initAprilTag();

        telemetry.addLine("Scan AprilTag in INIT loop");
        telemetry.update();

        // -------------------- INIT LOOP --------------------
        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                AprilTagDetection d = detections.get(0);
                artifactPattern = d.id;
                // remap pattern IDs if needed
                if (artifactPattern == 23) artifactPattern = 22;
                else if (artifactPattern == 21) artifactPattern = 23;
                else if (artifactPattern == 22) artifactPattern = 21;
                telemetry.addData("AprilTag detected", d.id);
                telemetry.addData("Mapped pattern", artifactPattern);

            } else {
                telemetry.addData("AprilTag", "none");
            }
            telemetry.update();
            sleep(50);
        }

        waitForStart();
        shooterTimer.reset();
        intake.setPower(0);

        // -------------------- Start Shooter Thread --------------------
        Thread shooterThread = new Thread(() -> {
            while (opModeIsActive() && shooterTimer.seconds() < 30) {
                shooter1.setVelocity(curTargetVelocity);

                shooter2.setVelocity(curTargetVelocity);

                shooter3.setVelocity(curTargetVelocity);





                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
            }
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooter3.setPower(0);
        });
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients1);
        shooter1.setVelocity(curTargetVelocity);

        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients2);
        shooter2.setVelocity(curTargetVelocity);

        shooter3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients3);
        shooter3.setVelocity(curTargetVelocity);
        shooterThread.setDaemon(true);
        shooterThread.start();

        // -------------------- RoadRunner Trajectories --------------------
        // Example coordinates — tune to match MeepMeep


        // Pickup artifact 1
        Actions.runBlocking(new SequentialAction(Everything.fire()));
        Actions.runBlocking(new SequentialAction(Everything.hood()));
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(-8, -25, Math.toRadians(-90)), Math.toRadians(50))
                        .build()
        );

        Actions.runBlocking(new ParallelAction(driveAction, Everything.roller()));
        Actions.runBlocking(
                drive.actionBuilder(pickup1end)
                        .setReversed(false)
                        .lineToY(-10)
                        .turn(Math.toRadians(-40.1))
                        .build()
        );
        Actions.runBlocking(new SequentialAction(Everything.fire()));


        // Move to firing position
        Actions.runBlocking(
                drive.actionBuilder(moveto2)
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(37, -20, Math.toRadians(-90)), Math.toRadians(50))
                        .build()
        );

        Actions.runBlocking(new ParallelAction(driveAction3, Everything.roller()));
        //  Actions.runBlocking(
        //          drive.actionBuilder(pickup2end)

        //                  .setReversed(true)
        //                  .lineToY(50)
        //                   .splineToSplineHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(50))
        //                  .build()
        //  );







        // -------------------- Firing Sequence --------------------
        // shooter1,2,3 fired flags

        //  for (ArtifactColor targetColor : requiredPattern) {
        //   boolean shotFired = false;

        // while (opModeIsActive() && !shotFired) {
        // read each shooter
        //     ArtifactColor s1 = detectColor(colorSensorI, colorSensorII);
        //       ArtifactColor s2 = detectColor(colorSensorIII, colorSensorIV);
        //      ArtifactColor s3 = detectColor(colorSensorV, colorSensorVI);

        //       if (!fired[0] && s1 == targetColor) {
        //         servoI.setPosition(0.9); sleep(300);
        //         servoI.setPosition(0.5); sleep(200);
        //              fired[0] = true;
        //            shotFired = true;
        //      } else if (!fired[1] && s2 == targetColor) {
        //        servoII.setPosition(0.9); sleep(300);
        //      servoII.setPosition(0.5); sleep(200);
        //        fired[1] = true;
        //shotFired = true;
        //  } else if (!fired[2] && s3 == targetColor) {
        //        servoIII.setPosition(0.9); sleep(300);
        //     servoIII.setPosition(0.5); sleep(200);
        //         fired[2] = true;
        //          shotFired = true;
        //  }

        // if (shotFired && firstShot) {
        // shift hood position for next shots
        //     leftHood.setPosition(hoodPosClose);
        //      rightHood.setPosition(hoodPosClose);
        //    firstShot = false;
        // }

        // Telemetry for alignment
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("CurrentPose", drive.localizer.getPose().toString());
        Pose2d diff = (drive.localizer.getPose());
        packet.put("x", diff.position.x);
        packet.put("y", diff.position.y);
        packet.put("θ", Math.toDegrees(diff.heading.toDouble()));

        telemetry.update();
    }
    //   }

    //      telemetry.addLine("Autonomous Complete");
    //       telemetry.update();
    //     sleep(1000);
    // }

    // -------------------- Utility Methods --------------------

    private void enableColorSensorLight(NormalizedColorSensor sensor) {
        if (sensor instanceof SwitchableLight) ((SwitchableLight) sensor).enableLight(true);
        sensor.setGain(2);
    }

    private ArtifactColor[] mapPattern(int id) {
        switch(id) {
            case 21: return new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            case 22: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
            case 23: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
            default: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
        }
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


}

