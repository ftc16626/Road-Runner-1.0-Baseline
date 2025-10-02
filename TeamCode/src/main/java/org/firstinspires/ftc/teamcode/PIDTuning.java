package org.firstinspires.ftc.teamcode;

import android.text.format.Time;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.tuning.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;



@Config
@TeleOp(name = "PIDTune")
public class PIDTuning extends OpMode {
    private PIDFController shooterController;

    public double Shooter_p = 0, Shooter_i = 0, Shooter_d = 0;
    public double Shooter_f = 0;
    final int COUNTS_PER_SHOOTER_MOTOR_REV = 28;

    final double     Shooter_WHEEL_DIAMETER_INCHES = 4 ;// For figuring circumference

    public int RPStarget = (96+(2/3)); //Maximum number of rotations in a second
    public double countsPerRPS = COUNTS_PER_SHOOTER_MOTOR_REV *RPStarget;
    private DcMotorEx ShooterMotor;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    @Override
    public void init() {

        shooterController = new PIDFController (Shooter_p,Shooter_i,Shooter_d,Shooter_f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ShooterMotor = hardwareMap.get(DcMotorEx.class,"Shooter");
    }

    @Override
    public void loop() {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        shooterController.setPIDF(Shooter_p,Shooter_i,Shooter_d,Shooter_f);

        int Shooterpos = ShooterMotor.getCurrentPosition();

        double Shooterpid = shooterController.calculate(Shooterpos, RPStarget);
        //double RPS = (double) countsPerRPS / runtime.time();
        double rot_power = Shooterpid;
        int checkInterval = 200;
        //int prevPosition = motor1.getCurrentPosition();




        telemetry.addData("Shooterpos", Shooterpos);
        telemetry.addData("Shootertarget", RPStarget);
        telemetry.update();





    }
}