package org.firstinspires.ftc.teamcode.ComponentSubClasses;

// All-purpose class for controlling the Illinois James configuration drive train
// for the 2025-2026 Decode Season

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    public DcMotor LFMotor = null;
    public DcMotor RFMotor = null;
    public DcMotor LBMotor = null;
    public DcMotor RBMotor = null;

    /// Method for powering wheels in TeleOp
    public void DrivePower(){
        double Drive = gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = gamepad1.left_stick_x;

        //Combines all output from joysticks to result in proper motor power allocation for movement
        double LFPower = Drive + Turn - Strafe;
        double RFPower = Drive + Turn + Strafe;
        double LBPower = Drive - Turn + Strafe;
        double RBPower = Drive - Turn - Strafe;
        double max;

        //Prevents max power values from exceeding -1 to 1 range
        max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
        max = Math.max(max, Math.abs(LBPower));
        max = Math.max(max, Math.abs(RBPower));

        if (max > 1.0) {
            LFPower /= max;
            RFPower /= max;
            LBPower /= max;
            RBPower /= max;
        }

        LFMotor.setPower(LFPower);
        RFMotor.setPower(RFPower);
        LBMotor.setPower(LBPower);
        RBMotor.setPower(RBPower);
    }

    /// Method used for setting up encoder motor mode
    public void setEncoder(){
            //Sets encoder value to 0
            LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Begins count of encoders
            LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /// Establishes relation between power output and wheel direction
    public void setDriveDirection(String LFDirection, String RFDirection,
                                  String LBDirection,String RBDirection){
        if (LFDirection == "FORWARD") {
            LFMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            LFMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (RFDirection == "FORWARD") {
            RFMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            RFMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (LBDirection == "FORWARD") {
            LBMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            LBMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if (RBDirection == "FORWARD") {
            RBMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            RBMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    /// Method used to configure the Drive Train according to DriverHub configuration
    public void getHardwareMap(HardwareMap hardwareMap){
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
    }
}
