package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class Shooter {

    public static double shooterSpeed = 0;
    public static double defaultShooterSpeed = 5000;
    public static double shooterPos = 0.6;
    DcMotorEx shooterL;
    DcMotorEx shooterR;
    Servo pivotL;
    Servo pivotR;
    TelemetryManager dashboardTelemetry;
    public Shooter(HardwareMap hardwareMap, TelemetryManager dashboard){
        shooterL = hardwareMap.get(DcMotorEx.class, "leftShooter");
        shooterR = hardwareMap.get(DcMotorEx.class, "rightShooter");
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotL = hardwareMap.get(Servo.class, "leftPivot");
        pivotR = hardwareMap.get(Servo.class, "rightPivot");
        pivotR.setDirection(Servo.Direction.REVERSE);
        dashboardTelemetry = dashboard;
    }

    public void setShooterSpeed(double p){
        shooterSpeed = p;
    }
    public void startShooter(){
        shooterSpeed = defaultShooterSpeed;
    }
    public void stopShooter(){
        shooterSpeed = 0;
    }

    public void setShooterPos(double pos){
        shooterPos = pos;
    }

    public void defaultShooterPos(){
        shooterPos = 0.6;
    }

    public void update(){
        double speed = shooterSpeed * 6;
        shooterL.setVelocity(speed, AngleUnit.DEGREES);
        shooterR.setVelocity(speed, AngleUnit.DEGREES);

        pivotL.setPosition(shooterPos);
        pivotR.setPosition(shooterPos);
        dashboardTelemetry.addData("shooterL Velocity", shooterR.getVelocity(AngleUnit.DEGREES)/6);
        dashboardTelemetry.addData("shooterR Velocity", shooterL.getVelocity(AngleUnit.DEGREES)/6);
    }
}
