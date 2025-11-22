package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class BasicTeleOp extends LinearOpMode {
    DcMotorEx LFMotor;
    DcMotorEx RFMotor;
    DcMotorEx LBMotor;
    DcMotorEx RBMotor;
    double drive;
    double turn;
    double strafe;
    double LFPower;
    double RFPower;
    double LBPower;
    double RBPower;

    @Override
    public void runOpMode(){
        LFMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotorEx.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotorEx.class, "RBMotor");

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);

        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()) {
            drive = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            LFPower = drive - turn - strafe;
            RFPower = drive + turn + strafe;
            LBPower = drive - turn + strafe;
            RBPower = drive + turn - strafe;

                    
            double driveTrainDenominator = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);


            LFMotor.setPower(LFPower / driveTrainDenominator);
            RFMotor.setPower(RFPower / driveTrainDenominator);
            LBMotor.setPower(LBPower / driveTrainDenominator);
            RBMotor.setPower(RBPower / driveTrainDenominator);
        }
    }

}