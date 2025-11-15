package org.firstinspires.ftc.teamcode.TeleOp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDTuning;
import org.firstinspires.ftc.teamcode.tuning.PIDFController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp (name = "pickme", group = "robot")
public class decodeskeletonthing extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor  rightFrontMotor;
    private DcMotor  rightBackMotor;
    private DcMotorEx shooter;
    //private CRServo conveyorServo;
    private Servo rightHoodServo;
    private Servo leftHoodServo;
    private CRServo rollerServo;
    private Servo armThing;
    private Servo flipper1;
    private Servo flipper2;
    private Servo flipper3;
    private final int READ_PERIOD = 1;
    private double Kp = 0.4;
    private double Ki = 0; // og Ki is 0.0008
    private double Kd = 0.1;
    private  double latestError;
    private double Sum;
    ElapsedTime timer = new ElapsedTime();
    double currentVelocity;
    public double targetRPM = 2300;
    public double servoPosition = 0.225;
    public double ticksPerRevolution = 28;
    public double targetVelocity = (targetRPM / 60) * ticksPerRevolution;
    private VisionPortal allSeeingEye;
    private AprilTagProcessor aprilTag;
    //private NormalizedColorSensor first;
  //  private NormalizedColorSensor second;
  //  private NormalizedColorSensor third;
  //  private NormalizedColorSensor fourth;
   // private NormalizedColorSensor fifth;
  //  private NormalizedColorSensor sixth;

    View relativeLayout;

    public void runOpMode() {
        initAprilTag();
        double leftFront;
        double leftBack;
        double rightFront;
        double rightBack;
        double drive;
        double turn;
        double strafe;
        double shoot;
        final double GRAVITY = 9.81;
        float gain = 2;


        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

     /*   if (first instanceof SwitchableLight) {
            ((SwitchableLight) first).enableLight(true);
        }
        if (second instanceof SwitchableLight) {
            ((SwitchableLight) second).enableLight(true);
        }

        if (third instanceof SwitchableLight) {
            ((SwitchableLight) third).enableLight(true);
        }
        if (fourth instanceof SwitchableLight) {
            ((SwitchableLight) fourth).enableLight(true);
        }
        if (fifth instanceof SwitchableLight) {
            ((SwitchableLight) fifth).enableLight(true);
        }

        if (sixth instanceof SwitchableLight) {
            ((SwitchableLight) sixth).enableLight(true);
        }
        final float[] hsvValues = new float[3];

      */


        // Define and Initialize Motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        // conveyorServo =  hardwareMap.get(CRServo.class, "conveyor");
        rollerServo = hardwareMap.get(CRServo.class, "roller");
        //armThing = hardwareMap.get(Servo.class, "armThing");
       // first = hardwareMap.get(NormalizedColorSensor.class, "first");
        //second = hardwareMap.get(NormalizedColorSensor.class, "first");
        //third = hardwareMap.get(NormalizedColorSensor.class, "third");
        //fourth = hardwareMap.get(NormalizedColorSensor.class, "fourth");
        //fifth = hardwareMap.get(NormalizedColorSensor.class, "fifth");
        //sixth = hardwareMap.get(NormalizedColorSensor.class, "sixth");
        flipper1 = hardwareMap.get(Servo.class, "flipper1");
        flipper2 = hardwareMap.get(Servo.class, "flipper2");
        flipper3 = hardwareMap.get(Servo.class, "flipper3");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHoodServo.setDirection(Servo.Direction.REVERSE);

       /* first.setGain(gain);
        second.setGain(gain);
        third.setGain(gain);
        fourth.setGain(gain);
        fifth.setGain(gain);
        sixth.setGain(gain);
        */
        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the allSeeingEye class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration allSeeingEye.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        // IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        //        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        //        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        // imu.initialize(parameters);
        // Define and initialize ALL installed servos.


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();
        leftHoodServo.setPosition(0);
        rightHoodServo.setPosition(0);
        double colorFind = 0;
        String obeliskCode;
        double areaOnex = 1.30;
        double areaOneAngle = Math.toRadians(79.967);
        double areaTwox = 0.675;
        double areaTwoAngle = Math.toRadians(71.184);
        double initialVelocity = 0; // Example initial velocity (m/s)
        double deltaX = areaOnex;          // Example horizontal distance (m)
        double theta = areaOneAngle; // Example launch angle (45 degrees)
        //double calculatedDeltaY = (deltaX * Math.tan(theta)) - (GRAVITY * Math.pow(deltaX, 2)) / (2 * Math.pow(initialVelocity * Math.cos(theta), 2));
        //telemetry.addData("Calculated Trajectory", calculatedDeltaY);
        telemetry.update();

        double targetDeltaX = areaOnex;    // Target horizontal distance (m)
        double targetDeltaY = 0.23;    // Target vertical distance (m)
        double launchAngle = Math.toRadians(30); // Target launch angle (30 degrees)
        // run until the end of the match (driver presses STOP)
        double numerator = GRAVITY * Math.pow(targetDeltaX, 2);
        double denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * (targetDeltaX * Math.tan(launchAngle) - targetDeltaY);
        if (denominator <= 0) {
            telemetry.addData("Error", "No valid initial velocity found for these parameters.");
            telemetry.update();
            return;
        }
        double initialVelocitySquared = numerator / denominator;
        double requiredInitialVelocity = Math.sqrt(initialVelocitySquared);
        telemetry.addData("Initial Velocity^2 (required)", initialVelocitySquared);
        telemetry.addData("Initial Velocity (required)", requiredInitialVelocity);
        telemetry.update();
        // Wait for telemetry to

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            telemetry.addData("DS preview on", "EasyOpenCV");
            telemetry.addData("Camera preview on", "Webcam");
            telemetry.update();

            waitForStart();


            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // Process detections
            if (gamepad1.square) {

                if (!currentDetections.isEmpty()) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            colorFind = detection.id;
                            telemetry.addData("ID", detection.id);
                            telemetry.addData("XYZ", detection.ftcPose.x + ", " + detection.ftcPose.y + ", " + detection.ftcPose.z);
                            telemetry.addData("Rotation", detection.ftcPose.roll + ", " + detection.ftcPose.pitch + ", " + detection.ftcPose.yaw);
                        }
                    }
                } else {
                    telemetry.addData("AprilTag", "Not detected");
                }
                telemetry.update();
            }

            allSeeingEye.close();


            double max;
            double driveTrainDenominator;
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;


            //      if (gamepad1.options) {
            //         imu.resetYaw();
            //   }

            // double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //     double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            //  double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing
            // Combine drive and turn for blended motion.


            // Normalize the values so neither exceed +/- 1.0
            leftFront = drive + turn - strafe;
            leftBack = drive + turn + strafe;
            rightFront = drive - turn + strafe;
            rightBack = drive - turn - strafe;

            driveTrainDenominator = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);

            if (gamepad2.dpad_left){
                targetRPM = 2300;
                targetVelocity = (targetRPM / 60) * ticksPerRevolution;
                servoPosition = 0.225;
            } else if (gamepad2.dpad_right){
                targetRPM = 3000;
                targetVelocity = (targetRPM / 60) * ticksPerRevolution;
                servoPosition = 0.4;
            }
//DO NOT GO HIGHER THAN 0.425 FOR HOOD SERVOS!!!!!!! YOU WILL HAVE TO PAY FOR DAMAGES ):<
        if (gamepad2.right_bumper) {
                rightHoodServo.setPosition(servoPosition);
                leftHoodServo.setPosition(servoPosition);
                shooter.setPower(PIDControl(targetVelocity, currentVelocity));
        } else if (gamepad2.left_bumper) {
            shooter.setPower(-0.25);
        } else {
            shooter.setPower(0);
        }

            if (gamepad2.dpad_up) {
                if (colorFind == 22) {
                    timer.reset();
                    flipper1.setPosition(0.9);
                    while (timer.milliseconds() < 500) {
                        flipper2.setPosition(0.47);
                    }
                    flipper2.setPosition(0.9);
                    while (timer.milliseconds() < 1000) {
                        flipper3.setPosition(0.53);
                    }
                    flipper3.setPosition(0.1);
                } else if (colorFind == 23) {
                    timer.reset();
                    flipper1.setPosition(0.9);
                    while (timer.milliseconds() < 500) {
                        flipper3.setPosition(0.53);
                    }
                    flipper3.setPosition(0.1);
                    while (timer.milliseconds() < 1000) {
                        flipper2.setPosition(0.49);
                    }
                    flipper3.setPosition(0.9);
                } else if (colorFind == 21) {
                    timer.reset();
                    flipper2.setPosition(0.9);
                    while (timer.milliseconds() < 500) {
                        flipper1.setPosition(0.47);
                    }
                    flipper1.setPosition(0.9);
                    while (timer.milliseconds() < 1000) {
                        flipper3.setPosition(0.53);
                    }
                    flipper3.setPosition(0.1);
                }
            }
        //shoot from closer zone
        //if (gamepad2.dpad_right){
        // targetDeltaX = areaTwox;
        //theta = areaTwoAngle;
        //numerator = GRAVITY * Math.pow(targetDeltaX, 2);
        //denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * (targetDeltaX * Math.tan(launchAngle) - targetDeltaY);
        //initialVelocitySquared = numerator / denominator;
        //requiredInitialVelocity = Math.sqrt(initialVelocitySquared);
        //shoot from farther zone
        // } else if (gamepad2.dpad_left){
        //targetDeltaX = areaOnex;
        //theta = areaOneAngle;
        //numerator = GRAVITY * Math.pow(targetDeltaX, 2);
        //denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * (targetDeltaX * Math.tan(launchAngle) - targetDeltaY);
        //initialVelocitySquared = numerator / denominator;
        //requiredInitialVelocity = Math.sqrt(initialVelocitySquared);
        //}


        // Output the safe vales to the motor drives.
        leftFrontMotor.setPower(leftFront / driveTrainDenominator);
        rightFrontMotor.setPower(rightFront / driveTrainDenominator);
        leftBackMotor.setPower(leftBack / driveTrainDenominator);
        rightBackMotor.setPower(rightBack / driveTrainDenominator);


        if (gamepad1.right_bumper) {
            rollerServo.setPower(1);
        } else if (gamepad1.left_bumper) {
            rollerServo.setPower(-1);

        } else {
            rollerServo.setPower(0);
        }

            if (gamepad1.dpad_up) {
                rightHoodServo.setPosition(0.225);
                leftHoodServo.setPosition(0.225);
            } else if (gamepad1.dpad_down) {
                rightHoodServo.setPosition(0.0);
                leftHoodServo.setPosition(0.0);
            }


        //rateLimit.reset();
        if (gamepad2.circle) {
            flipper3.setPosition(0.1);
            sleep(100);
            flipper3.setPosition(0.53);
        } else if (gamepad2.a) {
            flipper2.setPosition(0.9);
            sleep(100);
            flipper2.setPosition(0.47);
        } else if (gamepad2.square) {
            flipper1.setPosition(0.9);
            sleep(100);
            flipper1.setPosition(0.47);
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

         }


           /*/ NormalizedRGBA Cola1 = first.getNormalizedColors();
            Color.colorToHSV(Cola1.toColor(), hsvValues);
            NormalizedRGBA Cola2 = second.getNormalizedColors();
            Color.colorToHSV(Cola2.toColor(), hsvValues);
            NormalizedRGBA Cola3 = third.getNormalizedColors();
            Color.colorToHSV(Cola3.toColor(), hsvValues);
            NormalizedRGBA Cola4 = fourth.getNormalizedColors();
            Color.colorToHSV(Cola4.toColor(), hsvValues);
            NormalizedRGBA Cola5 = fifth.getNormalizedColors();
            Color.colorToHSV(Cola5.toColor(), hsvValues);
            NormalizedRGBA Cola6 = sixth.getNormalizedColors();
            Color.colorToHSV(Cola6.toColor(), hsvValues);/*/

        //if (gamepad1.right_bumper){
        // armThing.setPosition(1);
        //}
        // else if (gamepad1.left_bumper){
        //armThing.setPosition(0);
        //}


// This program shoots balls


        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.

        //conveyorServo.setPower(1);


        // Use gamepad left & right Bumpers to open and close the claw


        // Move both servos to new position.  Assume servos are mirror image of each other.


        // Use gamepad buttons to move arm up (Y) and down (A)


        // Send telemetry message to signify robot running;

        //Values of motor encoders, displayed on screen
        String LFEncoders = Integer.toString(leftFrontMotor.getCurrentPosition());
        String RFEncoders = Integer.toString(rightFrontMotor.getCurrentPosition());
        String LBEncoders = Integer.toString(leftBackMotor.getCurrentPosition());
        String RBEncoders = Integer.toString(rightBackMotor.getCurrentPosition());


        // Pace this loop so jaw action is reasonable speed.
        sleep(50);

     }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        allSeeingEye = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "allSeeingEye")) // Use your webcam's configured name
                .addProcessor(aprilTag)
                .build();
    }

    public double PIDControl (double reference, double state){
        currentVelocity = shooter.getVelocity();
        double deltaTime = timer.seconds();
        timer.reset();

        double error = targetVelocity - currentVelocity;
        Sum += error * deltaTime;
        latestError = error;
        double derivative = (error - latestError) / timer.seconds();
        if (currentVelocity >= ((targetRPM / 60) * ticksPerRevolution)){
            gamepad2.rumble(500);
        }
        timer.reset();

        double output = (error * Kp) + (derivative + Kd) + (Sum * Ki);
        return output;
    }
}

