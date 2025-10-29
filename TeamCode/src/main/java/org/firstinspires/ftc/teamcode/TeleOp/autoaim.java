package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp (name="autoaim", group="robot")
public class autoaim extends LinearOpMode{
    private final int READ_PERIOD = 1;
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor  rightFrontMotor;
    private DcMotor  rightBackMotor;

    private HuskyLens huskyLens;
    public void runOpMode() {
        double leftFront;
        double leftBack;
        double rightFront;
        double rightBack;
        double drive;
        double turn;
        double max;
        double strafe;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");


        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
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
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
        waitForStart();
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        // Combine drive and turn for blended motion.
        leftFront = drive + turn - strafe;
        leftBack = drive + turn + strafe;
        rightFront = drive - turn + strafe;
        rightBack = drive - turn - strafe;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
        max = Math.max(max, Math.abs(leftBack));
        max = Math.max(max, Math.abs(rightBack));
        if (max > 1.0) {
            leftFront /= max;
            rightFront /= max;
            leftBack /= max;
            rightBack /= max;
        }

        // Output the safe vales to the motor drives.
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftBackMotor.setPower(leftBack);
        rightBackMotor.setPower(rightBack);



        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
            HuskyLens.Block[] blocks = huskyLens.blocks(); // Get detected blocks

            if (blocks.length > 0) {
                HuskyLens.Block targetBlock = blocks[0]; // Assume tracking one object
                int centerhuskyX = 160;
                int centerhuskyY = 120;
                int Xposition = targetBlock.x;
                int Yposition = targetBlock.y;
                if (gamepad1.triangle) {
                    loop();
                    if (Xposition > centerhuskyX) {
                        leftFrontMotor.setPower(3.6);
                        leftBackMotor.setPower(-3.6);
                        rightFrontMotor.setPower(-3.6);
                        rightBackMotor.setPower(3.6);
                    } else if (Xposition < centerhuskyX) {
                        leftFrontMotor.setPower(-3.6);
                        leftBackMotor.setPower(3.6);
                        rightFrontMotor.setPower(3.6);
                        rightBackMotor.setPower(-3.6);
                    } else {
                        leftFrontMotor.setPower(0);
                        leftBackMotor.setPower(0);
                        rightFrontMotor.setPower(0);
                        rightBackMotor.setPower(0);
                    }
                    loop();
                    if (Yposition > centerhuskyY) {
                        leftFrontMotor.setPower(-0.1);
                        leftBackMotor.setPower(-0.1);
                        rightFrontMotor.setPower(-0.1);
                        rightBackMotor.setPower(-0.1);

                    } else if (Yposition < centerhuskyY) {
                        leftFrontMotor.setPower(0.1);
                        leftBackMotor.setPower(0.1);
                        rightFrontMotor.setPower(0.1);
                        rightBackMotor.setPower(0.1);

                    } else {
                        leftFrontMotor.setPower(0);
                        leftBackMotor.setPower(0);
                        rightFrontMotor.setPower(0);
                        rightBackMotor.setPower(0);

                    }
                }

            }


            telemetry.update();
        }
    }}



