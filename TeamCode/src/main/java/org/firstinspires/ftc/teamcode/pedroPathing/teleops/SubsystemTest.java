package org.firstinspires.ftc.teamcode.pedroPathing.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Ejector;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Index;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Shooter;

@Configurable       //if you want configurable constants
@TeleOp       //if this is a teleop
//@Autonomous   //if this is an auto
public class SubsystemTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Intake intake;
    Shooter shooter;
    Ejector ejector;
    Index index;
    //this section allows us to access telemetry data from a browser
    public static int targSlot = 0;
    PanelsTelemetry dashboard = PanelsTelemetry.INSTANCE;
    TelemetryManager dashboardTelemetry = dashboard.getTelemetry();
    public boolean logger = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        dashboardTelemetry.addData("Status", "Initialized");
        intake = new Intake(hardwareMap);
        ejector = new Ejector(hardwareMap);
        shooter = new Shooter(hardwareMap);
        index = new Index(hardwareMap, dashboardTelemetry);
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
        index.reset();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
//        if(index.colorIsDetected()) {
//            dashboardTelemetry.addData("sensed time", runtime.milliseconds());
//        }
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        index.toPickupTarget(targSlot);
        index.update();
        shooter.update();
        ejector.update();
        intake.update();
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
