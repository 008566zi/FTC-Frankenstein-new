package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * BaseClass is now the main TeleOp OpMode that runs the Drive and Linear Slide modules.
 */
@TeleOp(name = "Robot: Main Control", group = "TeleOp")
public class BaseClass extends LinearOpMode {

    // Drivetrain Motors
    protected DcMotor backLeftMotor = null;
    protected DcMotor backRightMotor = null;
    protected DcMotor frontLeftMotor = null;
    protected DcMotor frontRightMotor = null;

    // Linear Slide Motors
    protected DcMotor leftSlide = null;
    protected DcMotor rightSlide = null;

    // Component Instances
    private Drive driveModule;
    private Linear linearModule;

    // Global Variables
    public ElapsedTime matchTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize all hardware
        initHardware();

        // 2. Initialize the system modules
        driveModule = new Drive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        linearModule = new Linear(leftSlide, rightSlide);

        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();

        waitForStart();
        matchTime.reset();

        // 3. Main execution loop
        while (opModeIsActive()) {
            // Update components with gamepad inputs
            driveModule.update(gamepad1); // Left stick drives, bumpers scale speed
            linearModule.update(gamepad1); // Left stick Y controls manual lift, D-pad for presets

            // Display status and telemetry profiles
            telemetry.addData("Status", "Running");
            telemetry.addData("Drive Speed Modifier", "%.1f", driveModule.speedMultiplier);
            telemetry.addData("Slide Target Position", linearModule.getTargetPosition());
            telemetry.addData("Slide Real Pos", "L: %d | R: %d", 
                    linearModule.getLeftCurrentPosition(), linearModule.getRightCurrentPosition());
            telemetry.update();
        }

        // 4. Cleanup: Auto-retract linear slides when the user stops the OpMode
        telemetry.addData("Status", "Retracting Slides Safely...");
        telemetry.update();
        linearModule.retractOnStop();
    }

    /**
     * Maps hardware from the configuration and sets initial states.
     */
    public void initHardware() {
        // Hubs: Enable bulk caching for better performance
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Drivetrain: Map motors (make sure these names match your config!)
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        // Set drivetrain to run without encoders
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Slide Hardware Mapping
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        
        // Encoders initialization is fully handled internally by the Linear class constructor
    }
}
