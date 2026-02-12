package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyLimeLight {
    private final Limelight3A limelight;
    private double xLoc;
    private double yLoc;
    private double area;
    TelemetryManager dashboardTelemetry;

    public MyLimeLight(HardwareMap hardwareMap, TelemetryManager dashboard){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        dashboardTelemetry = dashboard;
    }

    public void start(int input){
        limelight.pipelineSwitch(input);
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }

    public boolean update(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Getting numbers from Python
            xLoc = result.getTx();
            yLoc = result.getTy();
            area = result.getTa();
            return true;
        }
        return false;
    }

    public double getxLoc(){
        return xLoc;
    }

    public double getyLoc(){
        return yLoc;
    }

    public double getAngle(){
        return area;
    }




}
