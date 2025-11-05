package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
public class PIDtest extends LinearOpMode {
    private DcMotor shooter;
    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double targetPosition = 6000;
    public double latestError = 0;
    public double Sum = 0;
    ElapsedTime timer = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public void runOpMode() {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        int targetPosition = 6000;
        while (opModeIsActive()){
            double power = PIDControl(targetPosition,shooter.getCurrentPosition());
            packet.put("power", power);
            packet.put("position", shooter.getCurrentPosition());
            packet.put("error", latestError);
            shooter.setPower(power);
            dashboard.sendTelemetryPacket(packet);

        }
    }



    public double PIDControl (double reference, double state){
        double error = reference - state;
        Sum += error * timer.seconds();
        double derivative = (error - latestError) / timer.seconds();
        latestError = error;
        timer.reset();

        double output = (error * Kp) + (derivative + Kd) + (Sum * Ki);
        return output;
    }

}
