package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

/**
 * Shooter subsystem.
 * Handles three shooter motors with independent PID control.
 */
public class ShooterSubsystemOdometryReady {

    public DcMotorEx shooter1, shooter2, shooter3;
    public Servo flipper1, flipper2, flipper3;


    public ShooterSubsystemOdometryReady(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        flipper1 = hardwareMap.get(Servo.class,"flipper1");
        flipper2 = hardwareMap.get(Servo.class,"flipper2");
        flipper3 = hardwareMap.get(Servo.class,"flipper3");


    }




    public void controlShooter(double curTargetVelocity, Gamepad gamepad) {
        if (gamepad.right_bumper) {
            double F1 = 13.9;
            double P1 = 80;
            shooter1.setVelocity(curTargetVelocity);
            double curVelocity1 = shooter1.getVelocity();
            double error1 = curTargetVelocity - curVelocity1;

            double F2 = 12.4;
            double P2 = 60;
            shooter2.setVelocity(curTargetVelocity);
            double curVelocity2 = shooter2.getVelocity();
            double error2 = curTargetVelocity - curVelocity2;

            double F3 = 12.045;
            double P3 = 60;
            shooter3.setVelocity(curTargetVelocity);
            double curVelocity3 = shooter2.getVelocity();
            double error3 = curTargetVelocity - curVelocity3;

            PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
            PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P2,0,0,F2);
            PIDFCoefficients pidfCoefficients3 = new PIDFCoefficients(P3,0,0,F3);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients1);
            shooter1.setVelocity(curTargetVelocity);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients2);
            shooter2.setVelocity(curTargetVelocity);
            shooter3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients3);
            shooter3.setVelocity(curTargetVelocity);


        } else if (gamepad.left_bumper) {
            flipper1.setPosition(0.6);
            flipper2.setPosition(0.6);
            flipper3.setPosition(0.41);
            curTargetVelocity = -500;
            double F1 = 13.9;
            double P1 = 80;
            shooter1.setVelocity(curTargetVelocity);
            double curVelocity1 = shooter1.getVelocity();
            double error1 = curTargetVelocity - curVelocity1;

            double F2 = 12.4;
            double P2 = 60;
            shooter2.setVelocity(curTargetVelocity);
            double curVelocity2 = shooter2.getVelocity();
            double error2 = curTargetVelocity - curVelocity2;

            double F3 = 12.045;
            double P3 = 60;
            shooter3.setVelocity(curTargetVelocity);
            double curVelocity3 = shooter2.getVelocity();
            double error3 = curTargetVelocity - curVelocity3;

            PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
            PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P2,0,0,F2);
            PIDFCoefficients pidfCoefficients3 = new PIDFCoefficients(P3,0,0,F3);
            shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients1);
            shooter1.setVelocity(curTargetVelocity);
            shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients2);
            shooter2.setVelocity(curTargetVelocity);
            shooter3.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients3);
            shooter3.setVelocity(curTargetVelocity);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooter3.setPower(0);
        }
    }



    }


