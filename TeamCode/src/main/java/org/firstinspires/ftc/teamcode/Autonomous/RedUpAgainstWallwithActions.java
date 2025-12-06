        package org.firstinspires.ftc.teamcode.Autonomous;
        import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

        import android.app.Activity;
        import android.view.View;

        import androidx.annotation.NonNull;
        import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
        import com.acmerobotics.roadrunner.Action;
        import com.acmerobotics.roadrunner.ParallelAction;
        import com.acmerobotics.roadrunner.SequentialAction;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
        import com.acmerobotics.roadrunner.ftc.Actions;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Gamepad;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
        import com.qualcomm.robotcore.hardware.NormalizedRGBA;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.vision.VisionPortal;
        import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
        import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// RR-specific imports
        import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.Vector2d;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.SwitchableLight;
        import com.qualcomm.robotcore.util.ElapsedTime;

// Non-RR imports
        import org.firstinspires.ftc.teamcode.MecanumDrive;

        import java.util.List;

        @Autonomous(name="TheRedUpAgainstWallAutoYouShouldUse", group="Robot")
public class RedUpAgainstWallwithActions extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightBackDrive  = null;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx shooter3;



    private  double latestError;
    private double Sum;
    ElapsedTime timer = new ElapsedTime();
    double currentVelocity;
    public double targetRPM = 2150;
    PIDControl pid1 = new PIDControl(shooter1);
    PIDControl pid2 = new PIDControl(shooter2);
    PIDControl pid3 = new PIDControl(shooter3);
    double power1 = pid1.update(targetRPM);
    double power2 = pid2.update(targetRPM);
    double power3 = pid3.update(targetRPM);

    public double ticksPerRevolution = 28;
    public double targetVelocity = (targetRPM / 60) * ticksPerRevolution;

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

    private Servo servoI;
    private Servo servoII;
    private Servo servoIII;
    private double done = 0;
    private double position = 0;

    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    enum State{
        Get_To_Power,
        Fling,
        Finished
    }
    State state = State.Get_To_Power;
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

    public class Shooter {
        private Servo servoI;
        private Servo servoII;
        private Servo servoIII;

        ElapsedTime timer;
        public Shooter (HardwareMap hardwareMapmap){
            shooter1  = hardwareMap.get(DcMotorEx.class, "shooter1");
            //leftFrontDrive = hardwareMap.get(DcMotor.class, "LFMotor");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "RFMotor");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "LBMotor");
            rightBackDrive = hardwareMap.get(DcMotor.class, "RBMotor");
            shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
            shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");
            servoI = hardwareMap.get(Servo.class, "flipper1");
            servoII = hardwareMap.get(Servo.class, "flipper2");
            servoIII = hardwareMap.get(Servo.class, "flipper3");
        }

        public Action shooterPower(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        encoderDrive(0,0,0,0,15,false,0,0.15,1.25);
                        while (currentVelocity < (targetVelocity + 500) && done!= 1) {
                            shooter1.setPower(power1);
                            shooter1.getVelocity();
                            if (currentVelocity >= targetVelocity){
                                if (artifactPattern == 21) {
                                    shooter1.setPower(power1);
                                    shooter2.setPower(power2);
                                    shooter3.setPower(power3);
                                    shooter1.getVelocity();
                                    servoII.setPosition(0.9);
                                    servoTimer.reset();
                                    shooter1.getVelocity();
                                    while (servoTimer.milliseconds() < 1000){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        shooter1.getVelocity();
                                        servoI.setPosition(0.47);
                                        shooter1.getVelocity();
                                    }
                                    servoI.setPosition(0.9);
                                    shooter1.getVelocity();
                                    while (servoTimer.milliseconds() < 1750){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        shooter1.getVelocity();
                                        servoIII.setPosition(0.53);
                                        shooter1.getVelocity();
                                    }
                                    servoIII.setPosition(0.1);
                                    shooter1.getVelocity();
                                    shooter1.setPower(0);
                                    shooter2.setPower(0);
                                    shooter3.setPower(0);
                                    done = 1;
                                } else if (artifactPattern == 22){
                                    shooter1.setPower(power1);
                                    shooter2.setPower(power2);
                                    shooter3.setPower(power3);
                                    shooter1.getVelocity();
                                    servoTimer.reset();
                                    servoI.setPosition(0.9);
                                    shooter1.getVelocity();
                                    while (servoTimer.milliseconds() < 1000){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        shooter1.getVelocity();
                                        servoII.setPosition(0.47);
                                        shooter1.getVelocity();
                                    }
                                    servoII.setPosition(0.9);
                                    while (servoTimer.milliseconds() < 1750){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        shooter1.getVelocity();
                                        servoIII.setPosition(0.51);
                                        shooter1.getVelocity();
                                    }
                                    servoIII.setPosition(0.1);
                                    shooter1.getVelocity();
                                    done = 1;
                                    shooter1.setPower(0);
                                    shooter2.setPower(0);
                                    shooter3.setPower(0);
                                } else if (artifactPattern == 23) {
                                    shooter1.setPower(power1);
                                    shooter2.setPower(power2);
                                    shooter3.setPower(power3);
                                    shooter1.getVelocity();
                                    servoI.setPosition(0.9);
                                    servoTimer.reset();
                                    shooter1.getVelocity();
                                    while (servoTimer.milliseconds() < 1000){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        shooter1.getVelocity();
                                        servoIII.setPosition(0.51);
                                        shooter1.getVelocity();
                                    }
                                    servoIII.setPosition(0.1);
                                    while (servoTimer.milliseconds() < 1750){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        shooter1.getVelocity();
                                        shooter1.getVelocity();
                                        servoII.setPosition(0.47);
                                    }
                                    servoII.setPosition(0.9);
                                    shooter1.getVelocity();
                                    shooter1.setPower(0);
                                    shooter2.setPower(0);
                                    shooter3.setPower(0);
                                    done = 1;
                                } else{
                                    servoTimer.reset();
                                    shooter1.setPower(power1);
                                    shooter2.setPower(power2);
                                    shooter3.setPower(power3);
                                    shooter1.getVelocity();
                                    servoII.setPosition(0.9);
                                    servoTimer.reset();
                                    shooter1.getVelocity();
                                    while (servoTimer.milliseconds() < 1000){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        shooter1.getVelocity();

                                        servoI.setPosition(0.47);
                                        shooter1.getVelocity();
                                    }
                                    servoI.setPosition(0.9);
                                    while (servoTimer.milliseconds() < 1750){
                                        shooter1.setPower(power1);
                                        shooter2.setPower(power2);
                                        shooter3.setPower(power3);
                                        servoIII.setPosition(0.53);
                                        shooter1.getVelocity();
                                    }
                                    servoIII.setPosition(0.1);
                                    shooter1.getVelocity();
                                    shooter1.setPower(0);
                                    shooter2.setPower(0);
                                    shooter3.setPower(0);
                                    done = 1;
                                    servoI.setPosition(0.47);
                                    servoII.setPosition(0.47);
                                    servoIII.setPosition(0.51);
                                }

                            }
                        }




                        initialized = true;

                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 2;
                }
            };
        }
        public Action Fire() {
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet){
                    if (!initialized){

                        if (artifactPattern == 21) {
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000) {
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500){
                                servoI.setPosition(0.47);
                            }
                            servoI.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000){
                                servoIII.setPosition(0.53);
                            }
                            servoIII.setPosition(0.1);
                            done = 1;
                        } else if (artifactPattern == 22){
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000){
                                servoI.setPosition(0.47);
                            }
                            servoI.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500){
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000){
                                servoIII.setPosition(0.51);
                            }
                            servoIII.setPosition(0.1);
                            done = 1;
                        } else if (artifactPattern == 23) {
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000){
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500){
                                servoI.setPosition(0.47);
                            }
                            servoIII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000){
                                servoIII.setPosition(0.51);
                            }
                            servoI.setPosition(0.1);
                            done = 1;
                        } else{
                            servoTimer.reset();
                            while (servoTimer.milliseconds() < 4000) {
                                servoII.setPosition(0.47);
                            }
                            servoII.setPosition(0.9);
                            while (servoTimer.milliseconds() < 4500){
                                servoI.setPosition(0.47);
                            }
                            servoI.setPosition(0.9);
                            while (servoTimer.milliseconds() < 5000){
                                servoIII.setPosition(0.53);
                            }
                            servoIII.setPosition(0.1);
                            done = 1;
                        }

                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 1;
                }
            };
        }
        public Action Scan(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                        if (!currentDetections.isEmpty()) {
                            for (AprilTagDetection detection : currentDetections) {
                                if (detection.metadata != null) {
                                    artifactPattern = detection.id;
                                    if (artifactPattern == 23){
                                        artifactPattern = 21;
                                    } else if (artifactPattern == 21) {
                                        artifactPattern = 22;
                                    } else if (artifactPattern == 22){
                                        artifactPattern = 23;
                                    }
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
                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 2;
                }
            };
        }
        public Action intake(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        encoderDrive(0,0,0,0,15,false,-1,0.15,5);
                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 2;
                }
            };
        }
        public Action goBack(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        encoderDrive(0.2,-3.25,-3.25,-3.25,-3.25,false,0,0.225,2);
                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 2;
                }
            };
        }
        public Action turn(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        encoderDrive(0.25,9.5,-9.5,9.5,-9.5,false,0,0,1);
                        encoderDrive(0.25,2,2,2,2,false,0,0,1);
                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 1;
                }
            };
        }
        public Action turn2(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        encoderDrive(0.25,7,7,7,77,true,0,0,2);
                        encoderDrive(0.25,-3.25,3.25,-3.25,3.25,false,0,0,2);
                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 2;
                }
            };
        }
        public Action outOfShootingArea(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        encoderDrive(0.2,3,3,3,3,false,0,0,2);
                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 2;
                }
            };
        }

        public Action strafeLeft(){
            return new Action() {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized){
                        encoderDrive(0.2,2,2,2,2,true,0,0,2);
                        initialized = true;
                        timer = new ElapsedTime();
                    }

                    return timer.seconds() < 2;
                }
            };
        }

    }



    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-52, 47, Math.toRadians(.26));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        initAprilTag();
        state = State.Get_To_Power;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        Shooter shoot = new Shooter(hardwareMap);
        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBMotor");
        intakeServo = hardwareMap.get(CRServo.class, "roller");
        rightHoodServo = hardwareMap.get(Servo.class, "rightHoodServo");
        leftHoodServo = hardwareMap.get(Servo.class, "leftHoodServo");
        servoI = hardwareMap.get(Servo.class, "flipper1");
        servoII = hardwareMap.get(Servo.class, "flipper2");
        servoIII = hardwareMap.get(Servo.class, "flipper3");
        colorSensorI = hardwareMap.get(NormalizedColorSensor.class, "first");
        colorSensorII = hardwareMap.get(NormalizedColorSensor.class, "second");
        colorSensorIII = hardwareMap.get(NormalizedColorSensor.class, "third");
        colorSensorIV = hardwareMap.get(NormalizedColorSensor.class, "fourth");
        colorSensorV = hardwareMap.get(NormalizedColorSensor.class, "fifth");
        colorSensorVI = hardwareMap.get(NormalizedColorSensor.class, "sixth");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter3.setDirection(DcMotorEx.Direction.FORWARD);
        leftHoodServo.setDirection(Servo.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-46.5281, -19.4465))
                .turn(Math.toRadians(-41.69))
                .waitSeconds(3);
        Pose2d newPose = new Pose2d(9.9556, -30.3882, Math.toRadians(-95.2059));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(newPose)
                .lineToY(-32.1531)
                .waitSeconds(3);
        Pose2d new2Pose =  new Pose2d(9.9556, -44.4149, Math.toRadians(-95.2059));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new2Pose)
                .strafeTo(new Vector2d(0, 0))
                .turn(-53.9244)
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(9.9556, -30.3882))
                .build();
        position = 1;
        Action trajectoryActionChosen;
        if (position == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (position == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }
        

        Actions.runBlocking(new SequentialAction(shoot.Scan()));
        Actions.runBlocking(new SequentialAction(shoot.shooterPower(), trajectoryActionChosen));
        position = 2;
        if (position == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (position == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }
        Actions.runBlocking(new SequentialAction(shoot.intake(), trajectoryActionChosen));
        position = 3;
        if (position == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (position == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }
        Actions.runBlocking(new SequentialAction(trajectoryActionChosen));
        Actions.runBlocking(new SequentialAction(trajectoryActionCloseOut));





        //Actions.runBlocking(new SequentialAction(shoot.outOfShootingArea()));
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
                             double leftBackInches, double rightBackInches, boolean strafe, double IntakePower, double Angulinator,
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


            /*ID21 = GPP | ID22 = PGP | ID23 = PPG*/
            /* Current paradigm has the middle servo (II) carrying green, rest are purple */



            if (strafe){
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
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

                leftHoodServo.setPosition(Angulinator);
                rightHoodServo.setPosition(Angulinator);
                intakeServo.setPower(IntakePower);

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
    public static final double TICKS_PER_REV = 28.0;
    private class PIDControl {
        private DcMotorEx shooter1;
        private double Kp = 8;
        private double Ki = 0.5;
        private double Kd = 1.3;

        private final ElapsedTime timer1 = new ElapsedTime();


        private double integral1 = 0.0;
        private double lastError1 = 0.0;
        private double integralLimit1 = 2000.0;

        private int lastPos1;
        private long lastTimeNano1;

        public PIDControl(DcMotorEx motor) {
            this.shooter1 = motor;
            this.lastPos1 = motor.getCurrentPosition();
            this.lastTimeNano1 = System.nanoTime();
            timer1.reset();
        }

        public void resetSampler() {
            lastPos1 = shooter1.getCurrentPosition();
            lastTimeNano1 = System.nanoTime();
            timer1.reset();
        }

        public void resetIntegral() {
            integral1 = 0.0;
            lastError1 = 0.0;
        }

        public double getRPM() {
            int curPos = shooter1.getCurrentPosition();
            long curTime = System.nanoTime();

            int deltaPos = curPos - lastPos1;
            long deltaNano = curTime - lastTimeNano1;
            if (deltaNano <= 0) deltaNano = 1;

            double seconds = deltaNano / 1e9;
            double ticksPerSec = deltaPos / seconds;
            double rpm = (ticksPerSec / TICKS_PER_REV) * 60.0;

            lastPos1 = curPos;
            lastTimeNano1 = curTime;

            return Math.abs(rpm);
        }

        public double update(double targetRPM) {
            double currentRPM = getRPM();

            double dt = timer1.seconds();
            timer1.reset();
            if (dt <= 0) dt = 0.001;

            double error = targetRPM - currentRPM;

            // integral with anti-windup
            integral1 += error * dt;
            if (integral1 > integralLimit1) integral1 = integralLimit1;
            if (integral1 < -integralLimit1) integral1 = -integralLimit1;

            double derivative = (error - lastError1) / dt;
            lastError1 = error;

            double out = Kp * error + Ki * integral1 + Kd * derivative;

            // clamp
            if (out < 0) out = 0;
            if (out > 1) out = 1;

            return out;
        }
        PIDControl pid1 = new PIDControl(shooter1);
        PIDControl pid2 = new PIDControl(shooter2);
        PIDControl pid3 = new PIDControl(shooter3);


    }
    }

