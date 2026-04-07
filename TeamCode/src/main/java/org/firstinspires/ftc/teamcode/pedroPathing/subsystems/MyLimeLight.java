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
    private double distance;
    double limelightMountAngleDegrees = 0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 17;

    // distance from the target to the floor
    double goalHeightInches = 30;
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
            double angleToGoalDegrees = limelightMountAngleDegrees + yLoc;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            this.distance = distanceFromLimelightToGoalInches;
            dashboardTelemetry.addData("xLoc", xLoc);
            dashboardTelemetry.addData("yLoc", yLoc);
            dashboardTelemetry.addData("Ta", area);
            dashboardTelemetry.addData("distance", distanceFromLimelightToGoalInches);
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

    public double getDistance(){
        return distance;
    }


}
