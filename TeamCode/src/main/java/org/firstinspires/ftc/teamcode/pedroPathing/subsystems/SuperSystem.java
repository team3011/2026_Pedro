package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SuperSystem {
    Index index;
    Ejector ejector;
    Intake intake;
    Shooter shooter;

    public static double shooterPower = 0.9;
    public SuperSystem(HardwareMap hardwareMap, TelemetryManager dashboard){
        index = new Index(hardwareMap, dashboard);
        ejector = new Ejector(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
    }

    public void reset(){
        index.reset();
    }

    public void shooterOn(){
        shooter.setShooterPos(shooterPower);
    }

    public void update(){
        index.update();
        ejector.update();
        intake.update();
        shooter.update();
    }
}
