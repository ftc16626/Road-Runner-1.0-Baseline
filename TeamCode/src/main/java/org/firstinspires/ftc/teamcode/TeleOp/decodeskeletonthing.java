package org.firstinspires.ftc.teamcode.TeleOp;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp (name = "decode skeleton code", group = "robot")
public class decodeskeletonthing extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor  rightFrontMotor;
    private DcMotor  rightBackMotor;
    private DcMotor shooter;
    private CRServo conveyorServo;
    private CRServo rollerServo;
    private Servo armThing;
    private Servo flipper1;
    private Servo flipper2;
    private Servo flipper3;
    private final int READ_PERIOD = 1;

    private HuskyLens allSeeingEye;
    private NormalizedColorSensor first;
    private NormalizedColorSensor second;
    private NormalizedColorSensor third;
    private NormalizedColorSensor fourth;
    private NormalizedColorSensor fifth;
    private NormalizedColorSensor sixth;
    View relativeLayout;

    public void runOpMode() {
        double leftFront;
        double leftBack;
        double rightFront;
        double rightBack;
        double drive;
        double turn;
        double max;
        double strafe;
        final double GRAVITY = 9.81;
        float gain = 2;

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        if (first instanceof SwitchableLight) {
            ((SwitchableLight)first).enableLight(true);
        }
        if (second instanceof SwitchableLight) {
            ((SwitchableLight)second).enableLight(true);
        }

        if (third instanceof SwitchableLight) {
            ((SwitchableLight)third).enableLight(true);
        }        if (fourth instanceof SwitchableLight) {
            ((SwitchableLight)fourth).enableLight(true);
        }
        if (fifth instanceof SwitchableLight) {
            ((SwitchableLight)fifth).enableLight(true);
        }

        if (sixth instanceof SwitchableLight) {
            ((SwitchableLight)sixth).enableLight(true);
        }
        final float[] hsvValues = new float[3];



        // Define and Initialize Motors
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackMotor  = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        conveyorServo =  hardwareMap.get(CRServo.class, "conveyor");
        rollerServo =  hardwareMap.get(CRServo.class, "roller");
        armThing = hardwareMap.get(Servo.class, "armThing");
        allSeeingEye = hardwareMap.get(HuskyLens.class, "allSeeingEye");
        first = hardwareMap.get(NormalizedColorSensor.class, "first");
        second = hardwareMap.get(NormalizedColorSensor.class, "first");
        third= hardwareMap.get(NormalizedColorSensor.class, "third");
        fourth = hardwareMap.get(NormalizedColorSensor.class, "fourth");
        fifth = hardwareMap.get(NormalizedColorSensor.class, "fifth");
        sixth = hardwareMap.get(NormalizedColorSensor.class, "sixth");
        flipper1 = hardwareMap.get(Servo.class, "flipper1");
        flipper2 = hardwareMap.get(Servo.class, "flipper2");
        flipper3 = hardwareMap.get(Servo.class, "flipper3");

        first.setGain(gain);
        second.setGain(gain);
        third.setGain(gain);
        fourth.setGain(gain);
        fifth.setGain(gain);
        sixth.setGain(gain);
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
        if (!allSeeingEye.knock()) {
            telemetry.addData(">>", "Problem communicating with " + allSeeingEye.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

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
        allSeeingEye.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        // Define and initialize ALL installed servos.


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();
        double colorFind = 0;
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
        sleep( 2000);
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

        sleep(10000); // Wait for telemetry to

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = allSeeingEye.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (gamepad1.square){
                    if (blocks[i].id == 1){
                        telemetry.addData("Obelisk", "PGP");
                        colorFind = 1;
                    } else if (blocks[i].id == 4){
                        telemetry.addData("Obelisk", "PPG");
                        colorFind = 2;
                    } else if (blocks[i].id == 5){
                        telemetry.addData("Obelisk", "GPP");
                        colorFind = 3;
                    }
                }

            }
            NormalizedRGBA Cola1 = first.getNormalizedColors();
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
            Color.colorToHSV(Cola6.toColor(), hsvValues);





            if (gamepad2.triangle){
                if (colorFind == 1) {
                    if (Cola1.blue >= 0.20 || Cola2.blue >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.blue >= 0.20 || Cola4.blue >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.blue >= 0.20 || Cola6.blue >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                    if (Cola1.green >= 0.20 || Cola2.green >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.green >= 0.20 || Cola4.green >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.green >= 0.20 || Cola6.green >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                    if (Cola1.blue >= 0.20 || Cola2. blue >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.blue >= 0.20 || Cola4.blue >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.blue >= 0.20 || Cola6.blue >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                if (colorFind == 2){
                    if (Cola1.blue >= 0.20 || Cola2.blue >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.blue >= 0.20 || Cola4.blue >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.blue >= 0.20 || Cola6.blue >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                    if (Cola1.blue >= 0.20 || Cola2.blue >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.blue >= 0.20 || Cola4.blue >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.blue >= 0.20 || Cola6.blue >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                    if (Cola1.green >= 0.20 || Cola2.green >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.green >= 0.20 || Cola4.green >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.green >= 0.20 || Cola6.green >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                }
                if (colorFind == 3){
                    if (Cola1.green >= 0.20 || Cola2.green >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.green >= 0.20 || Cola4.green >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.green >= 0.20 || Cola6.green >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                    if (Cola1.blue >= 0.20 || Cola2.blue >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.blue >= 0.20 || Cola4.blue >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.blue >= 0.20 || Cola6.blue >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                    if (Cola1.blue >= 0.20 || Cola2.blue >= 0.20) {
                        flipper1.setPosition(180);
                        flipper1.setPosition(0);
                    } else if (Cola3.blue >= 0.20 || Cola4.blue >= 0.20) {
                        flipper2.setPosition(180);
                        flipper2.setPosition(0);
                    }else if (Cola5.blue >= 0.20 || Cola6.blue >= 0.20) {
                        flipper3.setPosition(180);
                        flipper3.setPosition(0);
                    }
                }







            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing
            // Combine drive and turn for blended motion.


            // Normalize the values so neither exceed +/- 1.0
            leftFront  = drive + turn - strafe;
            leftBack = drive + turn + strafe;
            rightFront = drive - turn + strafe;
            rightBack = drive - turn - strafe;
            
            if (gamepad2.dpad_right){
                targetDeltaX = areaTwox;
                theta = areaTwoAngle;
                numerator = GRAVITY * Math.pow(targetDeltaX, 2);
                denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * (targetDeltaX * Math.tan(launchAngle) - targetDeltaY);
                initialVelocitySquared = numerator / denominator;
                requiredInitialVelocity = Math.sqrt(initialVelocitySquared);
            } else if (gamepad2.dpad_left){
                targetDeltaX = areaOnex;
                theta = areaOneAngle;
                numerator = GRAVITY * Math.pow(targetDeltaX, 2);
                denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * (targetDeltaX * Math.tan(launchAngle) - targetDeltaY);
                initialVelocitySquared = numerator / denominator;
                requiredInitialVelocity = Math.sqrt(initialVelocitySquared);
            }
            
            

          if (gamepad2.dpad_down){
              armThing.setPosition(180);
          } else if (gamepad2.dpad_up){
              armThing.setPosition(0);
          }

            // Output the safe vales to the motor drives.
            leftFrontMotor.setPower(leftFront);
            rightFrontMotor.setPower(rightFront);
            leftBackMotor.setPower(leftBack);
            rightBackMotor.setPower(rightBack);
            conveyorServo.setPower(1);
            rollerServo.setPower(1);
            shooter.setPower(requiredInitialVelocity);
            while (gamepad1.left_bumper){
                shooter.setPower(-1);  
            }


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
    }

}}}