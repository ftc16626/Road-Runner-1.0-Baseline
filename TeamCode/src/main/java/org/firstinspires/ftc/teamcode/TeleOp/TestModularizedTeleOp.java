package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ComponentSubClasses.DriveTrain;

@TeleOp
public class TestModularizedTeleOp extends LinearOpMode {
    //The variable with all information Drive Train in it
    DriveTrain GotsWheels;

    @Override
    public void runOpMode() {
        GotsWheels.setDriveDirection("REVERSE", "FORWARD", "REVERSE", "FORWARD");
        GotsWheels.getHardwareMap(hardwareMap);
        GotsWheels.setEncoder();
        while(opModeIsActive()) {
            GotsWheels.DrivePower();
        }
    }
}