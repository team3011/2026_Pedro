package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Index {
    public int GEARRATIO = 20;
    public int TICKSPERREV = 500;
    public int TICKSBETWEENSLOTS = TICKSPERREV/3;
    public static double maxPower = 0.5;
    public static double resetSpeed = 0.2;
    public static int targetPos;
    public static double indexPow;
    private DcMotorEx spindexer;
    private PIDController controller;
    private RevTouchSensor resetSensor;
    public static double kP = 0.04;
    public static double kI = 0.008;
    public static double kD = 0.0004;
    public boolean resetFlag = false;
    public boolean isMoving = false;
    PanelsTelemetry dashboard = PanelsTelemetry.INSTANCE;
    TelemetryManager dashboardTelemetry = dashboard.getTelemetry();
    public Index(HardwareMap hardwareMap){
        spindexer = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetSensor = hardwareMap.get(RevTouchSensor.class, "resetSensor");
        controller = new PIDController(kP, kI, kD);
    }

    public void toPickup(int slot){
        targetPos = TICKSBETWEENSLOTS*(slot-1);
    }

    public void toShoot(int slot){
        targetPos = (TICKSPERREV/2) + TICKSBETWEENSLOTS * (slot-1);
    }

    public void setMaxPower(double p){
        maxPower = p;
    }

    public void reset(){
        while(!resetSensor.isPressed()){
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spindexer.setPower(-resetSpeed);
        }
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(){
        double pid = 0;

        int currentPosition = spindexer.getCurrentPosition();
        if(currentPosition < targetPos-5 || currentPosition > targetPos+5){
            isMoving = true;
        }else{
            isMoving = false;
        }

        controller.setPID(kP, kI, kD);
        pid = this.controller.calculate(currentPosition, this.targetPos);
        pid = limiter(pid, maxPower);

        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setPower(pid);

        dashboardTelemetry.addData("current position in ticks", currentPosition);
        dashboardTelemetry.addData("target position in ticks", targetPos);
        dashboardTelemetry.addData("pid output", pid);
        dashboardTelemetry.addData("reset triggered", resetSensor.isPressed());
        dashboardTelemetry.addData("isMoving", isMoving);
    }

    private double limiter(double input, double limiter){
        if (input > limiter) {
            input = limiter;
        } else if (input < -limiter) {
            input = -limiter;
        }
        return input;
    }
}
