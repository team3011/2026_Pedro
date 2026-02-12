package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Index {
    public int GEARRATIO = 20;
    public int TICKSPERREV = 530;
    public int TICKSBETWEENSLOTS = TICKSPERREV/3;
    public int TICKSBETWEENLOCS = TICKSPERREV/6;
    public static double maxPower = 0.5;
    public static double minPower = 0.01;
    public static double resetSpeed = 0.3;
    public double currentPosInDegrees;
    public int currentRevs;
    public int currentPosition;
    public int currentSlot = 0; // slots go from 0-2
    public static int targetPos;
    private DcMotorEx spindexer;
    private PIDController controller;
    private RevTouchSensor resetSensor;
    private ColorSense colorSense;
    private Slot[]
            indexSlots = {new Slot(0, 0), new Slot(TICKSBETWEENSLOTS,0), new Slot(TICKSBETWEENSLOTS*2,0)};
    public static double kP = 0.012;
    public static double kI = 0.001;
    public static double kD = 0.0001;
    public boolean isSensing = false;
    public boolean resetFlag = false;
    public boolean isMoving = false;
    TelemetryManager dashboardTelemetry;
    public Index(HardwareMap hardwareMap, TelemetryManager dashboard){
        spindexer = hardwareMap.get(DcMotorEx.class, "spindex");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        resetSensor = hardwareMap.get(RevTouchSensor.class, "resetSensor");
        colorSense = new ColorSense(hardwareMap, dashboard);
        controller = new PIDController(kP, kI, kD);
        dashboardTelemetry = dashboard;
    }

    public void toPickup(int slot){
        int relativeTargetPos = (2*TICKSBETWEENLOCS*(slot)) + (TICKSBETWEENLOCS * 3);
        currentSlot = slot;
        if(currentPosition % TICKSPERREV < relativeTargetPos) {
            if (Math.abs(currentPosition - ((currentRevs * TICKSPERREV) + relativeTargetPos)) < Math.abs(currentPosition - (((currentRevs - 1) * TICKSPERREV) + relativeTargetPos))) {
                targetPos = (currentRevs * TICKSPERREV) + relativeTargetPos;
            }else{
                targetPos = ((currentRevs - 1) * TICKSPERREV) + relativeTargetPos;
            }
        }else{
            if (Math.abs(currentPosition - ((currentRevs * TICKSPERREV) + relativeTargetPos)) < Math.abs(currentPosition - (((currentRevs + 1) * TICKSPERREV) + relativeTargetPos))) {
                targetPos = (currentRevs * TICKSPERREV) + relativeTargetPos;
            }else{
                targetPos = ((currentRevs + 1) * TICKSPERREV) + relativeTargetPos;
            }
        }
    }
    public void toPickupTarget(int t){
        int closestSlot = -1;
        for(int i = 0; i<indexSlots.length; i++){
            int relativeTargetPos = (2*TICKSBETWEENLOCS*(i)) + (TICKSBETWEENLOCS * 3);
            int closestSlotPos = (2*TICKSBETWEENLOCS*(closestSlot) + (TICKSBETWEENLOCS * 3));
            if(indexSlots[i].getColor() == t && closestSlot == -1 || Math.abs(currentPosition-((currentRevs * TICKSPERREV) + relativeTargetPos)) < Math.abs(currentPosition-((currentRevs * TICKSPERREV) + closestSlotPos))){
                closestSlot = i;
            }
        }
        if(closestSlot == -1){
            targetPos = currentPosition;
        }else{
            toPickup(closestSlot);
        }
    }

    public void toShoot(int slot){
        int relativeTargetPos = 2 * TICKSBETWEENLOCS * (slot);
        currentSlot = slot;
        if(currentPosition % TICKSPERREV < relativeTargetPos) {
            if (Math.abs(currentPosition - ((currentRevs * TICKSPERREV) + relativeTargetPos)) < Math.abs(currentPosition - (((currentRevs - 1) * TICKSPERREV) + relativeTargetPos))) {
                targetPos = (currentRevs * TICKSPERREV) + relativeTargetPos;
            }else{
                targetPos = ((currentRevs - 1) * TICKSPERREV) + relativeTargetPos;
            }
        }else{
            if (Math.abs(currentPosition - ((currentRevs * TICKSPERREV) + relativeTargetPos)) < Math.abs(currentPosition - (((currentRevs + 1) * TICKSPERREV) + relativeTargetPos))) {
                targetPos = (currentRevs * TICKSPERREV) + relativeTargetPos;
            }else{
                targetPos = ((currentRevs + 1) * TICKSPERREV) + relativeTargetPos;
            }
        }
    }

    public void setMaxPower(double p){
        maxPower = p;
    }

    public void reset(){
        while(!resetSensor.isPressed()){
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spindexer.setPower(resetSpeed);
        }
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean ballDetected(){
        return colorSense.ballDetected();
    }
    public boolean currentSlotStatus(){
        return indexSlots[currentSlot].getColor() != 0;
    }
    public boolean isFull(){
        for(int i = 0; i<indexSlots.length; i++){
            if(indexSlots[i].getColor() == 0){
                return false;
            }
        }
        isSensing = false;
        return true;
    }
    public void setIsSensing(boolean b){
        isSensing = b;
    }

    public void update(){
        currentRevs = currentPosition/TICKSPERREV;
        colorSense.update();
        if(isSensing & colorSense.ballDetected()){
            indexSlots[currentSlot].setColor(colorSense.getColor());
        }
        double pid = 0;

        currentPosition = spindexer.getCurrentPosition();

        controller.setPID(kP, kI, kD);
        pid = this.controller.calculate(currentPosition, this.targetPos);
        pid = limiter(pid, maxPower);
        if(Math.abs(pid) < minPower){
            pid = 0;
        }

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
    public class Slot{
        private int position;
        private int color; // 0 = nothing, 1 = purple, 2 = green

        public Slot(int p, int c){
            position = p;
            color = c;
        }
        public int getPosition(){
            return position;
        }
        public int getColor(){
            return color;
        }
        public void setColor(int c){
            color = c;
        }
    }
}
