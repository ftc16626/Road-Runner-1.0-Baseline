package org.firstinspires.ftc.teamcode.ComponentSubClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    public ShooterPID pid1, pid2, pid3;

    public ShooterSubsystemOdometryReady(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter3.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pid1 = new ShooterPID(shooter1);
        pid2 = new ShooterPID(shooter2);
        pid3 = new ShooterPID(shooter3);
    }

    public void resetSamplers() {
        pid1.resetSampler();
        pid2.resetSampler();
        pid3.resetSampler();
    }

    public void setAllShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
        shooter3.setPower(power);
    }

    public void controlShooter(double targetRPM, Gamepad gamepad) {
        if (gamepad.right_bumper) {
            pid1.update(targetRPM, gamepad);
            pid2.update(targetRPM, gamepad);
            pid3.update(targetRPM, gamepad);
        } else if (gamepad.left_bumper) {
            setAllShooterPower(-0.25);
            pid1.resetIntegral();
            pid2.resetIntegral();
            pid3.resetIntegral();
        } else {
            setAllShooterPower(0.0);
            pid1.resetIntegral();
            pid2.resetIntegral();
            pid3.resetIntegral();
            pid1.resetRumble();
            pid2.resetRumble();
            pid3.resetRumble();
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("RPM1", (int) pid1.getRPM());
        telemetry.addData("RPM2", (int) pid2.getRPM());
        telemetry.addData("RPM3", (int) pid3.getRPM());
    }

    // -------------------- Shooter PID inner class --------------------
    public static class ShooterPID {
        private final DcMotorEx motor;
        private final ElapsedTime timer = new ElapsedTime();

        private double Kp = 8;
        private double Ki = 0.5;
        private double Kd = 1.3;

        private double integral = 0.0;
        private double lastError = 0.0;
        private double integralLimit = 2000.0;

        private int lastPos;
        private long lastTimeNano;
        private boolean hasRumbled = false;
        private double stableTimer = 0.0;
        private final double RPM_TOL = 60.0;
        private final double STABLE_REQUIRED = 0.25;

        public ShooterPID(DcMotorEx motor) {
            this.motor = motor;
            lastPos = motor.getCurrentPosition();
            lastTimeNano = System.nanoTime();
            timer.reset();
        }

        public void resetSampler() {
            lastPos = motor.getCurrentPosition();
            lastTimeNano = System.nanoTime();
            timer.reset();
        }

        public void resetIntegral() {
            integral = 0.0;
            lastError = 0.0;
        }

        public void resetRumble() {
            hasRumbled = false;
            stableTimer = 0.0;
        }

        public double getRPM() {
            int curPos = motor.getCurrentPosition();
            long curTime = System.nanoTime();
            int deltaPos = curPos - lastPos;
            long deltaNano = curTime - lastTimeNano;
            if (deltaNano <= 0) deltaNano = 1;
            double seconds = deltaNano / 1e9;
            double ticksPerSec = deltaPos / seconds;
            double rpm = (ticksPerSec / DriveSubsystemOdometryReady.TICKS_PER_REV) * 60.0;
            lastPos = curPos;
            lastTimeNano = curTime;
            return Math.abs(rpm);
        }

        public void update(double targetRPM, Gamepad gp) {
            double currentRPM = getRPM();
            double dt = timer.seconds();
            timer.reset();
            if (dt <= 0) dt = 0.001;

            double error = targetRPM - currentRPM;
            integral += error * dt;
            if (integral > integralLimit) integral = integralLimit;
            if (integral < -integralLimit) integral = -integralLimit;

            double derivative = (error - lastError) / dt;
            lastError = error;
            double out = Kp * error + Ki * integral + Kd * derivative;
            if (out < 0.0) out = 0.0;
            if (out > 1.0) out = 1.0;

            motor.setPower(out);

            if (Math.abs(currentRPM - targetRPM) <= RPM_TOL) {
                stableTimer += dt;
                hasRumbled = true;
            } else {
                stableTimer = 0.0;
                hasRumbled = false;
            }

            if (!hasRumbled && stableTimer >= STABLE_REQUIRED) {
                gp.rumble(0.7, 0.7, 300);
                hasRumbled = true;
            }
        }
    }
}

