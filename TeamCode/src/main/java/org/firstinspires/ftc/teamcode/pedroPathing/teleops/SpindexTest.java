package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable       //if you want config
@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public class SpindexTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //this section allows us to access telemetry data from a browser
    PanelsTelemetry dashboard = PanelsTelemetry.INSTANCE;
    TelemetryManager dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx spindexer;

    Servo hatch;

    public final double REVSPERTICK = 2800.0;

    public static double speed = 1;
    public static int tol = 1;
    public static int targetPos = 2800;

    public static double hatchPos = 1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dashboardTelemetry.addData("Status", "Initialized");

        hatch = hardwareMap.get(Servo.class, "hatch");
        hatch.setDirection(Servo.Direction.REVERSE);
        spindexer = hardwareMap.get(DcMotorEx.class, "indexer");
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        hatch.setPosition(hatchPos);
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("Ticks", spindexer.getCurrentPosition());
        dashboardTelemetry.addData("Speed", spindexer.getVelocity());
        dashboardTelemetry.addData("Degrees", spindexer.getCurrentPosition()/REVSPERTICK * 360);
        dashboardTelemetry.addData("Revs", spindexer.getCurrentPosition()/REVSPERTICK);
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
