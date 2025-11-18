package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.tuning.PIDFController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.tuning.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous(name = "PIDtest", group = "robot")
@Disabled
public class PIDtest extends LinearOpMode {
    private DcMotorEx shooter;
    public double Kp = 0.4;
    public double Ki = 0.0008;
    public double Kd = 0.1;
    public static double targetRPM = 3000;
    public static double ticksPerRevolution = 28;
    public double targetVelocity = (targetRPM / 60) * ticksPerRevolution;
    private final double TARGET_VELOCITY_TICKS_PER_SECOND = 1000;
    public double latestError = 0;
    public double Sum = 0;
    public double currentVelocity;
    ElapsedTime timer = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public void runOpMode() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()){
            double currentVelocity = shooter.getVelocity();
            double power = PIDControl(targetVelocity, currentVelocity);
            packet.put("Target Velocity", targetVelocity);
            packet.put("Current Velocity", currentVelocity );
            packet.put("error", latestError);
            shooter.setPower(power);
            dashboard.sendTelemetryPacket(packet);

        }
    }


    public double PIDControl (double reference, double state){
        currentVelocity = shooter.getVelocity();
        double deltaTime = timer.seconds();
        timer.reset();

        double error = targetVelocity - currentVelocity;
        Sum += error * deltaTime;
        latestError = error;
        double derivative = (error - latestError) / timer.seconds();
        timer.reset();

        double output = (error * Kp) + (derivative + Kd) + (Sum * Ki);
        return output;
    }

}
