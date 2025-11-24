
package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Drive subsystem for mecanum drive.
 * Odometry-ready: placeholders for left, right, front odometry wheels.
 */
public class DriveSubsystemOdometryReady {

    public DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    // public DcMotor leftOdom, rightOdom, frontOdom; // Uncomment for odometry

    public static final double TICKS_PER_REV = 28.0; // adjust if needed

    public DriveSubsystemOdometryReady(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveMecanum(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        double drive = -gamepad.left_stick_y;
        double strafe = -gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        double lf = drive + turn - strafe;
        double lb = drive + turn + strafe;
        double rf = drive - turn + strafe;
        double rb = drive - turn - strafe;

        double denom = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);

        leftFrontMotor.setPower(lf / denom);
        leftBackMotor.setPower(lb / denom);
        rightFrontMotor.setPower(rf / denom);
        rightBackMotor.setPower(rb / denom);
    }

    // -------------------- Odometry helper --------------------
    // public double ticksToInches(int ticks) {
    //     double WHEEL_DIAMETER_INCH = 2.0; // your odometry wheel
    //     double GEAR_RATIO = 1.0;
    //     return (ticks / TICKS_PER_REV) * Math.PI * WHEEL_DIAMETER_INCH * GEAR_RATIO;
    // }
}
