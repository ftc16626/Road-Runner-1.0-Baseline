package org.firstinspires.ftc.teamcode.Autonomous;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="RedDawn", group="Robot")
public class RedDawn extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightBackDrive  = null;
    private DcMotorEx shooter = null;
    private double Kp = 0.4;
    private double Ki = 0; // og Ki is 0.0008
    private double Kd = 0.1;
    private  double latestError;
    private double Sum;
    ElapsedTime timer = new ElapsedTime();
    double currentVelocity;
    public double targetRPM = 3000;
    public double ticksPerRevolution = 28;
    public double targetVelocity = (targetRPM / 60) * ticksPerRevolution;
    private Servo servoI;
    private Servo servoII;
    private Servo servoIII;
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
    double artifactPattern;



    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.141592653589);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    View relativeLayout;




    @Override
    public void runOpMode() {
        initAprilTag();

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBMotor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        servoI = hardwareMap.get(Servo.class, "flipper1");
        servoII = hardwareMap.get(Servo.class, "flipper2");
        servoIII = hardwareMap.get(Servo.class, "flipper3");
        intakeServo = hardwareMap.get(CRServo.class, "roller");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        colorSensorI = hardwareMap.get(NormalizedColorSensor.class, "first");
        colorSensorII = hardwareMap.get(NormalizedColorSensor.class, "second");
        colorSensorIII = hardwareMap.get(NormalizedColorSensor.class, "third");
        colorSensorIV = hardwareMap.get(NormalizedColorSensor.class, "fourth");
        colorSensorV = hardwareMap.get(NormalizedColorSensor.class, "fifth");
        colorSensorVI = hardwareMap.get(NormalizedColorSensor.class, "sixth");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        leftHoodServo.setDirection(Servo.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        if (colorSensorI instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorI).enableLight(true);
        }
        if (colorSensorII instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorII).enableLight(true);
        }
        if (colorSensorIII instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorIII).enableLight(true);
        }
        if (colorSensorIV instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorIV).enableLight(true);
        }
        if (colorSensorV instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorV).enableLight(true);
        }
        if (colorSensorVI instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorVI).enableLight(true);
        }

        colorSensorI.setGain(1);
        colorSensorII.setGain(1);
        colorSensorIII.setGain(1);
        colorSensorIV.setGain(1);
        colorSensorV.setGain(1);
        colorSensorVI.setGain(1);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition());
        telemetry.update();

        servoI.setPosition(0.5);
        servoII.setPosition(0.5);
        servoIII.setPosition(0.51);
        // Wait for the game to start (driver presses START)
        waitForStart();

        leftHoodServo.setPosition(0);
        rightHoodServo.setPosition(0);


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        allSeeingEye.close();


        encoderDrive(0.2,-12,-12,-12,-12, false, 0,false,false,0.4, PIDControl(targetVelocity,currentVelocity), 3); // Moving halfway forward
        encoderDrive(0,0,0,0,0, false, 0, true, false,0.4, PIDControl(targetVelocity,currentVelocity), 3); // Scan obelisk
        encoderDrive(0.2,-8,-8,-8,-8, false, 0,false,false,0.4,PIDControl(targetVelocity,currentVelocity), 2); // Finish moving forward
        encoderDrive(0.2, 7.8, -7.8, 7.8, -7.8, false,0, false,false,0.4, PIDControl(targetVelocity,currentVelocity),5); // Turn right ~45 degrees
        encoderDrive(0.2,-2.4,-2.4,-2.4,-2.4,false,0,false,false,0.4,PIDControl(targetVelocity,currentVelocity),5); // Move towards goal
        encoderDrive(0, 0, 0, 0, 0, false,0, false,true,0.4, PIDControl(targetVelocity,currentVelocity),7); // Launch artifacts
        encoderDrive(0.2, -7.8, 7.8, -7.8, 7.8, false,0, false,false, 0.4 ,0,5); // Turn left ~45 degrees
        encoderDrive(0.4, 6,6,6,6,false,0,false,false,0.4,0,3); // Move out of launch zone
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    ///Step used prior to EncoderDrive reference with Shooting = true, retrieves velocity artifact needs to travel
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches, boolean strafe, double IntakePower, boolean Sense, boolean Shooting, double Angulinator, double shootingPower,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        int shootingStep = 1;
        String ColorI = "Empty";
        String ColorII = "Empty";
        String ColorIII = "Empty";
        NormalizedRGBA colorsI = colorSensorI.getNormalizedColors();
        NormalizedRGBA colorsII = colorSensorII.getNormalizedColors();
        NormalizedRGBA colorsIII = colorSensorIII.getNormalizedColors();
        NormalizedRGBA colorsIV = colorSensorIV.getNormalizedColors();
        NormalizedRGBA colorsV = colorSensorV.getNormalizedColors();
        NormalizedRGBA colorsVI = colorSensorVI.getNormalizedColors();

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            if(Sense) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                if (!currentDetections.isEmpty()) {
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            artifactPattern = detection.id;
                            telemetry.addData("ID", detection.id);
                            telemetry.addData("XYZ", detection.ftcPose.x + ", " + detection.ftcPose.y + ", " + detection.ftcPose.z);
                            telemetry.addData("Rotation", detection.ftcPose.roll + ", " + detection.ftcPose.pitch + ", " + detection.ftcPose.yaw);
                        }
                    }
                } else {
                    telemetry.addData("AprilTag", "Not detected");
                }
                telemetry.update();
                allSeeingEye.close();
            }
            if(Shooting) {
                    /*ID21 = GPP | ID22 = PGP | ID23 = PPG*/
                /* Current paradigm has the middle servo (II) carrying green, rest are purple */
                if (artifactPattern == 21) {
                    servoII.setPosition(0.9);
                    sleep(500);
                    servoI.setPosition(0.9);
                    sleep(500);
                    servoIII.setPosition(0.1);
                } else if (artifactPattern == 22){
                    servoI.setPosition(0.9);
                    sleep(500);
                    servoII.setPosition(0.9);
                    sleep(500);
                    servoIII.setPosition(0.1);
                } else if (artifactPattern == 23) {
                    servoI.setPosition(0.9);
                    sleep(500);
                    servoIII.setPosition(0.1);
                    sleep(500);
                    servoII.setPosition(0.9);
                } else{
                    servoI.setPosition(0.9);
                    servoII.setPosition(0.9);
                    servoIII.setPosition(0.1);
                }
            }

            if (strafe){
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            } else {
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            }

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {


                intakeServo.setPower(IntakePower);
                rightHoodServo.setPosition(Angulinator);
                leftHoodServo.setPosition(Angulinator);
                shooter.setPower(shootingPower);
                if (colorsI.green > colorsI.blue || colorsII.green > colorsII.blue) {
                    ColorI = "Green";
                } else {
                    ColorI = "Purple";
                }
                if (colorsIII.green > colorsIII.blue || colorsIV.green > colorsIV.blue) {
                    ColorII = "Green";
                } else {
                    ColorII = "Purple";
                }
                if (colorsV.green > 0.5 && colorsV.blue < 0.5 || colorsVI.green > 0.5 && colorsVI.blue < 0.5) {
                    ColorIII = "Green";
                } else {
                    ColorIII = "Purple";
                }



                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.addLine()
                        .addData("Tag ID:", artifactPattern);
                telemetry.addLine()
                        .addData("ColorI:", ColorI);
                telemetry.addLine()
                        .addData("Red", colorsI.red)
                        .addData("Green", colorsI.green)
                        .addData("Blue", colorsI.blue);
                telemetry.update();
            }

            // Stop all motion;


            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            shooter.setPower(0);

            servoI.setPosition(0.5);
            servoII.setPosition(0.5);
            servoIII.setPosition(0.51);


            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
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