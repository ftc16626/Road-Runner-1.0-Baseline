package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem {
    private final Telemetry telemetry;
    private final DcMotor leftFront, rightFront, leftBack, rightBack;
    private final HardwareMap hw;

    // Encoder constants (kept from original)
    private static final double COUNTS_PER_MOTOR_REV = 384.5;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hw = hardwareMap;

        leftFront  = hw.get(DcMotor.class, "LFMotor");
        rightFront = hw.get(DcMotor.class, "RFMotor");
        leftBack   = hw.get(DcMotor.class, "LBMotor");
        rightBack  = hw.get(DcMotor.class, "RBMotor");

        // set directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // brake
        leftFront.setZeroPowerBehavior(BRAKE);
        leftBack.setZeroPowerBehavior(BRAKE);
        rightFront.setZeroPowerBehavior(BRAKE);
        rightBack.setZeroPowerBehavior(BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Blocking encoder drive using original signature logic.
     */
    public void encoderDriveBlocking(double speed,
                                     double leftFrontInches, double rightFrontInches,
                                     double leftBackInches, double rightBackInches,
                                     double timeoutS,
                                     boolean strafe,
                                     double intakePower,
                                     double angulinator) {
        if (speed == 0) return;

        if (strafe) {
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
        } else {
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        }

        int newLF = leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
        int newRF = rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
        int newLB = leftBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
        int newRB = rightBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newLF);
        rightFront.setTargetPosition(newRF);
        leftBack.setTargetPosition(newLB);
        rightBack.setTargetPosition(newRB);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())
                && timer.seconds() < timeoutS) {
            // lightweight telemetry
            if ((int)(timer.milliseconds()) % 200 < 20) {
                telemetry.addData("LF pos", leftFront.getCurrentPosition());
                telemetry.update();
            }
            // allow external yields (the opmode will call idle implicitly)
        }

        // stop motors and restore modes
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

