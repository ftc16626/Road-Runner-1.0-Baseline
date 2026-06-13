package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ComponentSubClasses.IntakeSubsystemOdometryReady;


@TeleOp (name = "FieldCentric")
public class FieldCentricGo extends LinearOpMode {
    private IntakeSubsystemOdometryReady intake;
    @Override
    public void runOpMode() {

        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        intake = new IntakeSubsystemOdometryReady(hardwareMap);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            intake.controlIntake(gamepad1);
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            double denom = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1.0);
            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double adjustedStrafe = -drive * Math.sin(heading) + strafe * Math.cos(heading);
            double adjustedDrive = drive * Math.cos(heading) + strafe * Math.sin(heading);
            leftFrontMotor.setPower(adjustedDrive + adjustedStrafe + turn / denom);
            leftBackMotor.setPower(adjustedDrive - adjustedStrafe + turn / denom);
            rightFrontMotor.setPower(adjustedDrive - adjustedStrafe - turn / denom);
            rightBackMotor.setPower(adjustedDrive + adjustedStrafe -turn / denom);

            if (gamepad1.options){
                imu.resetYaw();
            }
        }

    }
}
