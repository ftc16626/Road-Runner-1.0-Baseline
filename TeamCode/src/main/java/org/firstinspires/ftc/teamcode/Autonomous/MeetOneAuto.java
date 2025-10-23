package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MeetOneAuto", group="Robot")
public class MeetOneAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightBackDrive  = null;
    private DcMotor         shooter = null;
    private Servo servoI;
    private Servo servoII;
    private Servo servoIII;
    private CRServo intakeServoI;
    private CRServo intakeServoII;
    private NormalizedColorSensor colorSensorI;
    private NormalizedColorSensor colorSensorII;
    private NormalizedColorSensor colorSensorIII;
    private NormalizedColorSensor colorSensorIV;
    private NormalizedColorSensor colorSensorV;
    private NormalizedColorSensor colorSensorVI;
    private HuskyLens huskyLens;



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
    public String artifactPattern;



    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBMotor");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        servoI = hardwareMap.get(Servo.class, "flipper1");
        servoII = hardwareMap.get(Servo.class, "flipper2");
        servoIII = hardwareMap.get(Servo.class, "flipper3");
        intakeServoI = hardwareMap.get(CRServo.class, "");
        intakeServoII = hardwareMap.get(CRServo.class, "");
        colorSensorI = hardwareMap.get(NormalizedColorSensor.class, "first");
        colorSensorII = hardwareMap.get(NormalizedColorSensor.class, "second");
        colorSensorIII = hardwareMap.get(NormalizedColorSensor.class, "third");
        colorSensorIV = hardwareMap.get(NormalizedColorSensor.class, "fourth");
        colorSensorV = hardwareMap.get(NormalizedColorSensor.class, "fifth");
        colorSensorVI = hardwareMap.get(NormalizedColorSensor.class, "sixth");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        // Wait for the game to start (driver presses START)
        waitForStart();
        HuskyLens.Block block = null;
        //PPG = 4 GPP = 5 PGP = 1
        if (block.id == 1) {
            artifactPattern = "PGP";
        }
        if(block.id == 4) {
            artifactPattern = "PPG";
        }
        if(block.id == 5) {
            artifactPattern = "GPP";
        }





        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //encoderDrive(DRIVE_SPEED,   72, 72, 72, 72, false, 0, 5);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(1, -13.5, 13.5, -13.5, 13.5, false, 0,3);  // S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(0,0,0,0,0, false, 0,true,0.6,5);
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
    public double getRequiredInitialVelocity(double launchAngle,
                                           double targetDeltaX, double targetDeltaY){
        double GRAVITY = 9.81; //  m/s^2
        launchAngle = Math.toRadians(launchAngle);
        double numerator = GRAVITY * Math.pow(targetDeltaX, 2); //Delta X: horizontal distance from target, Y: vert. distance
        double denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * (targetDeltaX * Math.tan(launchAngle) - targetDeltaY);
        double initialVelocitySquared = numerator/denominator;
        double requiredInitialVelocity = Math.sqrt(initialVelocitySquared);
        return requiredInitialVelocity;
    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches, boolean strafe, double IntakePower, boolean Shooting, double shooterPower,
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


                intakeServoI.setPower(IntakePower);
                intakeServoII.setPower(-IntakePower);

                if (colorsI.green > 0.5 && colorsI.blue < 0.5 || colorsII.green > 0.5 && colorsII.blue < 0.5) {
                    ColorI = "Green";
                } else {
                    ColorI = "Purple";
                }
                if (colorsIII.green > 0.5 && colorsIII.blue < 0.5 || colorsIV.green > 0.5 && colorsIV.blue < 0.5) {
                    ColorII = "Green";
                } else {
                    ColorII = "Purple";
                }
                if (colorsV.green > 0.5 && colorsV.blue < 0.5 || colorsVI.green > 0.5 && colorsVI.blue < 0.5) {
                    ColorIII = "Green";
                } else {
                    ColorIII = "Purple";
                }

                if(Shooting) {
                    if (artifactPattern == "PGP") {
                            //Shooting the first artifact of the pattern
                            if (ColorI == "Purple" && shootingStep == 1) {
                                shooter.setPower(shooterPower);
                                servoI.setPosition(0.1);
                                shootingStep += 1;
                            }
                            if (ColorII == "Purple" && shootingStep == 1) {
                                shooter.setPower(shooterPower);
                                servoII.setPosition(0.9);
                                shootingStep += 1;
                            }
                            if (ColorIII == "Purple" && shootingStep == 1) {
                                shooter.setPower(shooterPower);
                                servoIII.setPosition(0.9);
                                shootingStep += 1;
                            }

                            //Shooting the second artifact of the pattern
                            if (ColorI == "Green" && shootingStep == 2) {
                                    shooter.setPower(shooterPower);
                                    servoI.setPosition(0.1);
                                    shootingStep += 1;
                            }
                            if (ColorII == "Green" && shootingStep == 2) {
                                    shooter.setPower(shooterPower);
                                    servoII.setPosition(0.9);
                                    shootingStep += 1;
                            }
                            if (ColorIII == "Green" && shootingStep == 2) {
                                    shooter.setPower(shooterPower);
                                    servoIII.setPosition(0.9);
                                    shootingStep += 1;
                            }

                        //Shooting the third artifact of the pattern
                        if (ColorI == "Purple" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoI.setPosition(0.1);
                            shootingStep = 1;
                        }
                        if (ColorII == "Purple" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoII.setPosition(0.9);
                            shootingStep = 1;
                        }
                        if (ColorIII == "Purple" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoIII.setPosition(0.9);
                            shootingStep = 1;
                        }

                    }
                    if (artifactPattern == "PPG"){
                        if (ColorI == "Purple" && shootingStep == 1) {
                            shooter.setPower(shooterPower);
                            servoI.setPosition(0.1);
                            shootingStep += 1;
                        }
                        if (ColorII == "Purple" && shootingStep == 1) {
                            shooter.setPower(shooterPower);
                            servoII.setPosition(0.9);
                            shootingStep += 1;
                        }
                        if (ColorIII == "Purple" && shootingStep == 1) {
                            shooter.setPower(shooterPower);
                            servoIII.setPosition(0.9);
                            shootingStep += 1;
                        }

                        if (ColorI == "Purple" && shootingStep == 2) {
                            shooter.setPower(shooterPower);
                            servoI.setPosition(0.1);
                            shootingStep += 1;
                        }
                        if (ColorII == "Purple" && shootingStep == 2) {
                            shooter.setPower(shooterPower);
                            servoII.setPosition(0.9);
                            shootingStep += 1;
                        }
                        if (ColorIII == "Purple" && shootingStep == 2) {
                            shooter.setPower(shooterPower);
                            servoIII.setPosition(0.9);
                            shootingStep += 1;
                        }

                        if (ColorI == "Green" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoI.setPosition(0.1);
                            shootingStep = 1;
                        }
                        if (ColorII == "Green" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoII.setPosition(0.9);
                            shootingStep = 1;
                        }
                        if (ColorIII == "Green" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoIII.setPosition(0.9);
                            shootingStep = 1;
                        }
                    }
                    if (artifactPattern == "GPP"){
                        if (ColorI == "Green" && shootingStep == 1) {
                            shooter.setPower(shooterPower);
                            servoI.setPosition(0.1);
                            shootingStep += 1;
                        }
                        if (ColorII == "Green" && shootingStep == 1) {
                            shooter.setPower(shooterPower);
                            servoII.setPosition(0.9);
                            shootingStep += 1;
                        }
                        if (ColorIII == "Green" && shootingStep == 1) {
                            shooter.setPower(shooterPower);
                            servoIII.setPosition(0.9);
                            shootingStep += 1;
                        }

                        if (ColorI == "Purple" && shootingStep == 2) {
                            shooter.setPower(shooterPower);
                            servoI.setPosition(0.1);
                            shootingStep += 1;
                        }
                        if (ColorII == "Purple" && shootingStep == 2) {
                            shooter.setPower(shooterPower);
                            servoII.setPosition(0.9);
                            shootingStep += 1;
                        }
                        if (ColorIII == "Purple" && shootingStep == 2) {
                            shooter.setPower(shooterPower);
                            servoIII.setPosition(0.9);
                            shootingStep += 1;
                        }

                        if (ColorI == "Purple" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoI.setPosition(0.1);
                            shootingStep = 1;
                        }
                        if (ColorII == "Purple" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoII.setPosition(0.9);
                            shootingStep = 1;
                        }
                        if (ColorIII == "Purple" && shootingStep == 3) {
                            shooter.setPower(shooterPower);
                            servoIII.setPosition(0.9);
                            shootingStep = 1;
                        }
                    }
                }



                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget, newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
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


            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}

