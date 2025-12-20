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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.lang.Math;
import java.util.List;

@Autonomous(name="RedUpAgainstTheGoalRR", group="Robot")
public class RenameTest extends LinearOpMode {

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

    private PIDControl pid1, pid2, pid3;
    private ElapsedTime shooterTimer = new ElapsedTime();

    private double hoodPosClose = 0.225;
    private double targetRPM = 2050;

    private int artifactPattern = 0;
    public class Everything {
        private CRServo intake;
        private Servo servoI, servoII, servoIII;
        private NormalizedColorSensor colorSensorI, colorSensorII;
        private NormalizedColorSensor colorSensorIII, colorSensorIV;
        private NormalizedColorSensor colorSensorV, colorSensorVI;

        ElapsedTime timer;

        public Everything(HardwareMap hardwareMapmap) {

            intake = hardwareMap.get(CRServo.class, "roller");

            servoI = hardwareMap.get(Servo.class, "flipper3");
            servoII = hardwareMap.get(Servo.class, "flipper2");
            servoIII = hardwareMap.get(Servo.class, "flipper1");

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

        public Action roller() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        intake.setPower(-1);
                        initialized = true;
                        timer = new ElapsedTime();
                    }
                    return timer.seconds() < 2;
                }
            };
        }

        public Action fire() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        ArtifactColor[] requiredPattern = mapPattern(artifactPattern);

                        boolean firstShot = true;
                        boolean[] fired = {false, false, false}; // shooter1,2,3 fired flags

                        for (ArtifactColor targetColor : requiredPattern) {
                            boolean shotFired = false;

                            while (opModeIsActive() && !shotFired) {
                                // read each shooter
                                ArtifactColor s1 = detectColor(colorSensorI, colorSensorII);
                                ArtifactColor s2 = detectColor(colorSensorIII, colorSensorIV);
                                ArtifactColor s3 = detectColor(colorSensorV, colorSensorVI);

                                if (!fired[0] && s1 == targetColor) {
                                    servoI.setPosition(0.9);
                                    sleep(300);
                                    servoI.setPosition(0.5);
                                    sleep(200);
                                    fired[0] = true;
                                    shotFired = true;
                                } else if (!fired[1] && s2 == targetColor) {
                                    servoII.setPosition(0.9);
                                    sleep(300);
                                    servoII.setPosition(0.5);
                                    sleep(200);
                                    fired[1] = true;
                                    shotFired = true;
                                } else if (!fired[2] && s3 == targetColor) {
                                    servoIII.setPosition(0.9);
                                    sleep(300);
                                    servoIII.setPosition(0.5);
                                    sleep(200);
                                    fired[2] = true;
                                    shotFired = true;
                                }

                                if (shotFired && firstShot) {
                                    // shift hood position for next shots
                                    leftHood.setPosition(hoodPosClose);
                                    rightHood.setPosition(hoodPosClose);
                                    firstShot = false;
                                }


                                timer = new ElapsedTime();
                                initialized = true;
                            }

                        }

                    }
                    return timer.seconds() < 20;
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


        Pose2d startPose = new Pose2d(-51, 50, Math.toRadians(135));
        Pose2d pickup1 = (new Pose2d(-8, 30, Math.toRadians(90.9)));
        Pose2d pickup1end = (new Pose2d(-8, 57, Math.toRadians(90.9)));
        Pose2d moveto2 = (new Pose2d(-8, 10, Math.toRadians(135)));
        Pose2d pickup2 = (new Pose2d(14.5, 30, Math.toRadians(90.9)));
        Pose2d pickup2end = (new Pose2d(14.5, 60, Math.toRadians(90.9)));
        Pose2d pickup3 = (new Pose2d(38.5, 30, Math.toRadians(90.9)));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action driveAction = drive.actionBuilder(pickup1)
            .lineToY(57)
                .build();
        Action driveAction2 = drive.actionBuilder(pickup2)
                .lineToY(60)
                .build();
        Action driveAction3 = drive.actionBuilder(pickup3)
                .lineToY(60)
                .build();


        // -------------------- Initialize Hardware --------------------
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");
        Everything Everything = new Everything(hardwareMap);



        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter3.setDirection(DcMotorEx.Direction.FORWARD);

        pid1 = new PIDControl(shooter1, 28);
        pid2 = new PIDControl(shooter2, 28);
        pid3 = new PIDControl(shooter3, 28);

     //   servoI = hardwareMap.get(Servo.class, "flipper3");
     //   servoII = hardwareMap.get(Servo.class, "flipper2");
      //  servoIII = hardwareMap.get(Servo.class, "flipper1");
       // Intake = hardwareMap.get(Servo.class, "roller");

        leftHood = hardwareMap.get(Servo.class, "leftHoodServo");
        rightHood = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHood.setDirection(Servo.Direction.REVERSE);
        leftHood.setPosition(0.0);
        rightHood.setPosition(0.0);



     /*   colorSensorI = hardwareMap.get(NormalizedColorSensor.class, "first");
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
        enableColorSensorLight(colorSensorVI); */

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
                if (artifactPattern == 23) artifactPattern = 21;
                else if (artifactPattern == 21) artifactPattern = 22;
                else if (artifactPattern == 22) artifactPattern = 23;
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

        // -------------------- Start Shooter Thread --------------------
        Thread shooterThread = new Thread(() -> {
            while (opModeIsActive() && shooterTimer.seconds() < 30) {
                double out1 = pid1.update(targetRPM);
                double out2 = pid2.update(targetRPM);
                double out3 = pid3.update(targetRPM);

                shooter1.setPower(out1);
                shooter2.setPower(out2);
                shooter3.setPower(out3);

                try { Thread.sleep(20); } catch (InterruptedException e) { break; }
            }
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooter3.setPower(0);
        });
        shooterThread.setDaemon(true);
        shooterThread.start();

        // -------------------- RoadRunner Trajectories --------------------
        // Example coordinates — tune to match MeepMeep


        // Pickup artifact 1
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(-8,30))
                        .turn(Math.toRadians(-44.1))
                        .waitSeconds(1)
                        .build()
        );
        Actions.runBlocking(new ParallelAction(driveAction, Everything.roller()));
        Actions.runBlocking(
                drive.actionBuilder(pickup1end)
                        .lineToY(10)
                        .turn(Math.toRadians(44.1))
                        .waitSeconds(1)
                        .build()
        );
        Actions.runBlocking(new SequentialAction(Everything.fire()));


        // Move to firing position
        Actions.runBlocking(
                drive.actionBuilder(moveto2)
                        .strafeTo(new Vector2d(14.5,30))
                        .turn(Math.toRadians(-44.1))
                        .waitSeconds(1)
                        .build()
        );
        Actions.runBlocking(new ParallelAction(driveAction2, Everything.roller()));
        Actions.runBlocking(
                drive.actionBuilder(pickup2end)
                        .strafeTo(new Vector2d(-8,10))
                        .turn(Math.toRadians(44.1))
                        .waitSeconds(1)
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(moveto2)
                        .strafeTo(new Vector2d(38.5,30))
                        .turn(Math.toRadians(-44.1))
                        .waitSeconds(1)
                        .build()
        );
        Actions.runBlocking(new ParallelAction(driveAction3, Everything.roller()));







        // -------------------- Firing Sequence --------------------
        ArtifactColor[] requiredPattern = mapPattern(artifactPattern);

        boolean firstShot = true;
        boolean[] fired = {false, false, false}; // shooter1,2,3 fired flags

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
        sensor.setGain(1);
    }

    private ArtifactColor[] mapPattern(int id) {
        switch(id) {
            case 21: return new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
            case 22: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE};
            case 23: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN};
            default: return new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.PURPLE};
        }
    }

    private ArtifactColor detectColor(NormalizedColorSensor s1, NormalizedColorSensor s2) {
        NormalizedRGBA c1 = s1.getNormalizedColors();
        NormalizedRGBA c2 = s2.getNormalizedColors();

        double r = (c1.red + c2.red)/2.0;
        double g = (c1.green + c2.green)/2.0;
        double b = (c1.blue + c2.blue)/2.0;

        // robust classifier
        if (g > r + 0.03 && g > b + 0.03) return ArtifactColor.GREEN;
        else if ((r + b) > 2.0 * g) return ArtifactColor.PURPLE;
        else return ArtifactColor.NONE;
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

    // -------------------- PID Control --------------------
    private static class PIDControl {
        private final DcMotorEx motor;
        private final double ticksPerRev;
        private double Kp = 8.0, Ki = 0.5, Kd = 1.3;
        private double integral = 0.0, lastError = 0.0;
        private final double integralLimit = 2000.0;
        private int lastPos;
        private long lastTimeNs;
        private double lastRPM = 0.0;

        public PIDControl(DcMotorEx motor, double ticksPerRev) {
            this.motor = motor;
            this.ticksPerRev = ticksPerRev;
            this.lastPos = motor.getCurrentPosition();
            this.lastTimeNs = System.nanoTime();
        }

        public synchronized double update(double targetRPM) {
            int curPos = motor.getCurrentPosition();
            long now = System.nanoTime();
            int dPos = curPos - lastPos;
            long dNs = Math.max(1, now - lastTimeNs);
            double dt = dNs/1e9;

            double currentRPM = Math.abs((dPos/dt)/ticksPerRev*60.0);
            lastRPM = 0.7*lastRPM + 0.3*currentRPM;

            double error = targetRPM - lastRPM;
            integral += error*dt;
            integral = Math.max(-integralLimit, Math.min(integral, integralLimit));
            double derivative = (error - lastError)/dt;
            if (Math.abs(derivative) > 5000) derivative = 0.0;
            lastError = error;

            double output = Kp*error + Ki*integral + Kd*derivative;
            return Math.max(0.0, Math.min(1.0, output));
        }
    }
}
