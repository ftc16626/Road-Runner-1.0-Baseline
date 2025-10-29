package org.firstinspires.ftc.teamcode.TeleOp;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class TestDrive extends LinearOpMode {
    DcMotor LFMotor = null;
    DcMotor LBMotor = null;
    DcMotor RFMotor = null;
    DcMotor RBMotor = null;
    @Override
    public void runOpMode() {
        double leftFront;
        double leftBack;
        double rightFront;
        double rightBack;
        double drive;
        double turn;
        double max;
        double strafe;

        // Define and Initialize Motors
        LFMotor  = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor  = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            // Combine drive and turn for blended motion.
            leftFront  = drive + turn - strafe;
            leftBack = drive + turn + strafe;
            rightFront = drive - turn + strafe;
            rightBack = drive - turn - strafe;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            max = Math.max(max, Math.abs(leftBack));
            max = Math.max(max, Math.abs(rightBack));
            if (max > 1.0)
            {
                leftFront /= max;
                rightFront /= max;
                leftBack /= max;
                rightBack /= max;
            }

            // Output the safe vales to the motor drives.
            LFMotor.setPower(leftFront);
            RFMotor.setPower(rightFront);
            LBMotor.setPower(leftBack);
            RBMotor.setPower(rightBack);


            // Use gamepad left & right Bumpers to open and close the claw


            // Move both servos to new position.  Assume servos are mirror image of each other.


            // Use gamepad buttons to move arm up (Y) and down (A)


            // Send telemetry message to signify robot running;

            //Values of motor encoders, displayed on screen
            String LFEncoders = Integer.toString(LFMotor.getCurrentPosition());
            String RFEncoders = Integer.toString(RFMotor.getCurrentPosition());
            String LBEncoders = Integer.toString(LBMotor.getCurrentPosition());
            String RBEncoders = Integer.toString(RBMotor.getCurrentPosition());


            telemetry.addData("leftFrontPower",  "%.2f", leftFront);
            telemetry.addData("rightFrontPower", "%.2f", rightFront);
            telemetry.addData("leftBackPower",  "%.2f", leftBack);
            telemetry.addData("rightBackPower", "%.2f", rightBack);
            telemetry.addData("Left Front Encoder:", LFEncoders);
            telemetry.addData("Right Front Encoder:", RFEncoders);
            telemetry.addData("Left Back Encoder:", LBEncoders);
            telemetry.addData("Right Back Encoder:", RBEncoders);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
