package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "PIDtest", group = "robot")
public class PIDtest extends LinearOpMode {


    // ----- PID Constants -----
    public static double Kp1 = 0;
    public static double Ki1 = 0;
    public static double Kd1 = 0;
    public static double Kp2 = 0;
    public static double Ki2 = 0;
    public static double Kd2 = 0;
    public static double Kp3 = 0;
    public static double Ki3 = 0;
    public static double Kd3 = 0;

    public static double targetVelocity = 2300;

    // ----- Shooter Motors -----
    private DcMotorEx shooter1, shooter2, shooter3;



    // ----- PID states for each motor -----
    PIDState1 pid1 = new PIDState1();
    PIDState2 pid2 = new PIDState2();
    PIDState3 pid3 = new PIDState3();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {

        // ---- Motor Init ----
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter3 = hardwareMap.get(DcMotorEx.class, "shooter3");


        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setDirection(DcMotorEx.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




        // Ensure PID timers start at 0
        pid1.timer.reset();
        pid2.timer.reset();
        pid3.timer.reset();

        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            // ---- Read velocities ----
            double v1 = shooter1.getVelocity();
            double v2 = shooter2.getVelocity();
            double v3 = shooter3.getVelocity();

            // ---- Compute PID ----
            double power1 = PID1(targetVelocity, v1, pid1);
            double power2 = PID2(targetVelocity, v2, pid2);
            double power3 = PID3(targetVelocity, v3, pid3);

            // ---- Apply power ----
            shooter1.setPower(power1);
            shooter2.setPower(power2);
            shooter3.setPower(power3);

            // ---- Telemetry to Dashboard ----
            packet.put("Target Velocity", targetVelocity);

            packet.put("Velocity1", v1);

            packet.put("Error1", pid1.lastError1);

            packet.put("Velocity2", v2);

            packet.put("Error2", pid2.lastError2);

            packet.put("Velocity3", v3);

            packet.put("Error3", pid3.lastError3);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    // ----- PID Function -----
    public double PID1(double reference1, double state1, PIDState1 pid) {

        double error1 = reference1 - state1;

        double dt = pid.timer.seconds();
        pid.timer.reset();

        if (dt == 0) dt = 0.001; // prevent div by zero

        pid.integral1 += error1 * dt;
        double derivative1 = (error1 - pid.lastError1) / dt;

        pid.lastError1 = error1;
        double output1 = (Kp1 * error1) + (Ki1 * pid.integral1) + (Kd1 * derivative1);

// Clamp to motor power limits
        output1 = Math.max(-1, Math.min(1, output1));
        return output1;
    }

    // ----- PID State Class -----
    static class PIDState1 {
        public double lastError1 = 0;
        public double integral1 = 0;
        public ElapsedTime timer = new ElapsedTime();
    }
    public double PID2(double reference2, double state2, PIDState2 pid) {

        double error2 = reference2 - state2;

        double dt = pid.timer.seconds();
        pid.timer.reset();

        if (dt == 0) dt = 0.001; // prevent div by zero

        pid.integral2 += error2 * dt;
        double derivative2 = (error2 - pid.lastError2) / dt;

        pid.lastError2 = error2;

        double output2 = (Kp2 * error2) + (Ki2 * pid.integral2) + (Kd2 * derivative2);

// Clamp to motor power limits
        output2 = Math.max(-1, Math.min(1, output2));
        return output2;
    }

    // ----- PID State Class -----
    static class PIDState2 {
        public double lastError2 = 0;
        public double integral2 = 0;
        public ElapsedTime timer = new ElapsedTime();
    }
    public double PID3(double reference3, double state3, PIDState3 pid) {

        double error3 = reference3 - state3;

        double dt = pid.timer.seconds();
        pid.timer.reset();

        if (dt == 0) dt = 0.001; // prevent div by zero

        pid.integral3 += error3 * dt;
        double derivative3 = (error3 - pid.lastError3) / dt;

        pid.lastError3 = error3;

        double output3 = (Kp3 * error3) + (Ki3 * pid.integral3) + (Kd3 * derivative3);

// Clamp to motor power limits
        output3 = Math.max(-1, Math.min(1, output3));
        return output3;
    }

    // ----- PID State Class -----
    static class PIDState3 {
        public double lastError3 = 0;
        public double integral3 = 0;
        public ElapsedTime timer = new ElapsedTime();
    }
}
