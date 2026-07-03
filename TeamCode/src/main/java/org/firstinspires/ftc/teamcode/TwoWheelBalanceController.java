
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Balance Controller class for a two wheel balancing robot.
 * Defines the motors and odometry (for position and velocity) for each wheel.
 * Uses four terms (states) to control balance: Position, Velocity, Pitch, PitchRate
 * Also provides Yaw (turn) control using a PID.
 * IMU provides pitch, pitchRate, Yaw and YawRate
 * The Position and velocity is of the center of the wheels.
 */
public class TwoWheelBalanceController {
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;

    private final TWBOdometry odometry; // two wheel odometry object with running average
    int leftTicks = 0;
    int rightTicks = 0;
    private int leftZeroTicks = 0; // pinpoint does not reset.  have to store zero at start
    private int rightZeroTicks = 0; // pinpoint does not reset.  have to store zero at start

    // These are the state terms for a two wheel balancing robot
    private double Kpitch = -0.0001; // volts/degree
    private double KpitchRate = -0.0001; // volts/degrees/sec

    // Both Kpos and Kvelo are negative when the center of mass is below the wheel axles
    // and positive when the CM is above (unstable)
    private double Kpos = 0.0001;  // volts/mm
    private double Kvelo = 0.0001;  // volts/mm/sec

    private double TICKSPERMM = 1; // set in initialization

    private boolean revEncoders = false; // reverse sign of encoders?

    // YAW PID
    private final PIDController yawPID;

    private double posTarget = 0.0;
    private double sOdom = 0.0; // Current robot position from odometry

    private final double veloTarget = 0.0; // not using velocity target
    private double linearVelocity = 0.0;

    private double vertCM = 10.0;  // vertical distance mm from the wheel center to the robot center of mass
    private double autoPitchTarget = 0; // used to set pitch from an auto routine
    private double armPitchTarget = 0;
    private double pitchTarget = 0;

    private double pitch = 0;
    private double oldPitch = 0;
    private double pitchRATE = 0;

    private double yawTarget = 0.0;
    private double yaw = 0;
    private double priorYaw = 0;
    double rawYaw = 0;
    private double rawPriorYaw = 0;
    double yawRate = 0;

    public IMU imu;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private YawPitchRollAngles orientation;   // part of FIRST navigation classes

    private double positionVolts = 0.0;
    private double pitchVolts = 0.0;

    private final ElapsedTime runtime = new ElapsedTime(); // Timer used to get loop times

    // The variables below are to try to get a consistent delta time for the controller.
    // Not sure how well this works. Don't know how to make it better without different runtime env.
    // Array size was 11 for IMU.  8 with gobilda pinpoint.
    private final RunningAverageArray deltaTimeRA = new RunningAverageArray(20,false);
    private double currentTime;
    private double lastTime;
    private double deltaTime = 0.04; // initialize, replaced by a running average
    public double MMPLoop = 5.0; // 8 is large for pinpoint
    public double DEGPLoop = 0.0;

    private DatalogTWB datalogTWB; // datalog for full recording
    private boolean writeDatalog = false; // default is no log.  call method to write.
    private final boolean fixedLoopTime = false; // not using fixed loop times

    /**
     * TWB Constructor.  Call once in initialization.
     * Sign convention: L -^- R : + dist + velocity as shown.
     * Motors and Encoders both have sign!
     * Pitch: + pitch + pitch_rate is "nose" up.
     * Yaw:  L - +CCW - R  + yaw + yaw_rate is CCW from above.
     * @param hardwareMap hardware map
     * @param wheelBase Distance between Wheels in mm
     * @param ticksPerMM Odometry ticks per mm of wheel travel
     * @param kp Yaw PID Kp term
     * @param ki Yaw PID Ki term
     * @param kd Yaw PID Kd term
     * @param NVelo Size of running average array for robot velocity
     * @param NDist Size of running average array for robot odometry distance
      */
    public TwoWheelBalanceController(HardwareMap hardwareMap, double wheelBase,
                                     double ticksPerMM, double kp, double ki, double kd,
                                     int NVelo, int NDist) {

        deltaTimeRA.add(0.04); // add to running average to smooth the start??

        // Define and Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        imu = hardwareMap.get(IMU.class, "imu");

        odometry = new TWBOdometry(wheelBase, getPitch(),NVelo,NDist); // create odometry object
        TICKSPERMM = ticksPerMM;

        yawPID = new PIDController(kp, ki, kd);

        yawPID.setSetpoint(0.0);    // initial yaw (yawTarget) is zero.
    }
    public void initializePinpoint(HardwareMap hardwareMap) {
        // initialize the Pinpoint, that has an IMU
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(0.0, 0.0, DistanceUnit.MM);
        odo.setEncoderResolution(27.16244, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU(); // recalibrates IMU
        leftZeroTicks = odo.getEncoderX();
        rightZeroTicks = odo.getEncoderY();
    }
    public void setDriveMotors(boolean leftForward, boolean rightForward, boolean reverseEncoders) {
        if(leftForward) leftDrive.setDirection(DcMotor.Direction.FORWARD);
        else leftDrive.setDirection(DcMotor.Direction.REVERSE);

        if(rightForward) rightDrive.setDirection(DcMotor.Direction.FORWARD);
        else rightDrive.setDirection(DcMotor.Direction.REVERSE);

        resetMotors();
        this.revEncoders = reverseEncoders;
    }

    /**
     * setBalanceTerms initializes the four balance controller terms
     * @param kpos K Position  volts/mm
     * @param kvelo K Velocity volts/mm/second
     * @param kpitch K Pitch   volts/degree
     * @param kpitchrate K Pitch Rate  volts/degree/second
     */
    public void setBalanceTerms(double kpos, double kvelo, double kpitch, double kpitchrate) {
        Kpos = kpos;
        Kvelo = kvelo;
        Kpitch = kpitch;
        KpitchRate = kpitchrate;
    }

    /**
     * TWB start method. Called once on Start press. Resets encoders, timers, PIDs
      */
    public void start() {
        resetMotors();

        // reset the timer
        runtime.reset();
        currentTime = runtime.seconds();
        lastTime = currentTime;

        // reset the PIDs
        yawPID.reset();
    }

    public void zeroPinpointTicks() {
        leftZeroTicks = odo.getEncoderX();
        rightZeroTicks = odo.getEncoderY();
    }
    private void resetMotors() {
        // reset the encoders
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // reset the motors
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
     * TWB Main Loop method.  Call repeatedly while running. Contains balance control logic.
     * Teleoperated inputs are removed from this method, so it can be called in autonomous.
      */
    public void loop(OpMode theOpmode) {
        if (fixedLoopTime) deltaTime = 0.015; // experiment to see if we can make velocity smoother
        else setLoopTime(); // this updates the deltaTime value

        updateTicks();

        updatePitchYaw();

        // update position and linear velocity values from wheel encoders (odometry)
        odometry.update(leftTicks / TICKSPERMM,
                rightTicks / TICKSPERMM, vertCM, pitch, deltaTime);
        sOdom = odometry.getS();  // position
        linearVelocity = odometry.getAvgLinearVelocity();

        // MAIN BALANCE CONTROL CODE:
        double posError = sOdom - posTarget;
        positionVolts = Kvelo * linearVelocity + Kpos * posError;

        pitchTarget = armPitchTarget + autoPitchTarget;
        double pitchError = pitch - pitchTarget;

        pitchVolts = Kpitch * pitchError + KpitchRate * pitchRATE;
        double totalPowerVolts = pitchVolts + positionVolts;

        makeYawContinuous();

        yawPID.setSetpoint(yawTarget);
        double yawPower = yawPID.compute(yaw,yawRate);

         // Set the motor power for both wheels
        leftDrive.setPower(totalPowerVolts  - yawPower);
        rightDrive.setPower(totalPowerVolts  + yawPower);

        // kill the robot if it pitches over too far or runs fast when not asked to
        if ((Math.abs(pitch) > 60.0)  || (Math.abs(linearVelocity) > 1400)) {
            theOpmode.requestOpModeStop(); // Stop the opmode
        }

        if (writeDatalog) {
            datalogTWB.logPosPitch(getPos(), getPosTarget(),
                    getVelocity(), getVeloTarget(),getPitch(),
                    getPitchTarget(), getPitchRate(), getYaw(),getYawTarget(),
                    getPositionVolts(),getPitchVolts(), getDeltaTime());
            datalogTWB.writeLineTWB();
        }
    }
    public void makeYawContinuous() {
        // The following controls the turn (yaw) of the robot
        // IMU getYaw always returns value from -2*PI to 2*PI
        // The code below makes "yaw" a continuous value
        double deltaYaw = rawYaw - rawPriorYaw;
        rawPriorYaw = rawYaw;
        if (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        else if (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;
        yaw = priorYaw + deltaYaw;
        priorYaw = yaw;
    }
    public void setMotorsZero() {
        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }
    public void imuYawPitchReset() {
        // Doesn't seem to be working if robot yaw is greater than 180
        imu.resetYaw();
        yawTarget = 0.0;
        yaw = 0.0;
        priorYaw = 0.0;
        rawPriorYaw = 0.0;
        yawPID.reset();
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
    }
    private void setLoopTime () {
        // compute a loop time.  Using running average to smooth values
        lastTime = currentTime;
        currentTime = runtime.seconds();
        double dT = currentTime - lastTime;
        if(dT > 0.07) dT = 0.07; // fake!
        // add the new delta time to the running average
        deltaTimeRA.add(dT);
        deltaTime = deltaTimeRA.getAverage();
    }
    public void updateTicks() {
        if (revEncoders) {
            leftTicks = -leftDrive.getCurrentPosition();
            rightTicks = -rightDrive.getCurrentPosition();
        } else {
            leftTicks = leftDrive.getCurrentPosition();
            rightTicks = rightDrive.getCurrentPosition();
        }
    }
    public void updateTicksPinpoint() {
        odo.update(); // Update the pinpoint values for the following calls
        leftTicks = odo.getEncoderX()-leftZeroTicks;
        rightTicks = odo.getEncoderY()-rightZeroTicks;
    }
    public void updatePitchYaw() {
        // get pitch and pitch rate values from the IMU
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);

        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        pitchRATE = angularVelocity.xRotationRate;

        rawYaw = orientation.getYaw(AngleUnit.RADIANS);
    }
    public void updatePitchYawPinpoint() {
        pitch = odo.getPitch(AngleUnit.DEGREES);
        pitchRATE = (pitch- oldPitch)/deltaTime;
        oldPitch = pitch;

        yaw = -odo.getHeading(UnnormalizedAngleUnit.RADIANS);
        yawRate = odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }
    public void writeDatalog(String LogName) {
        this.writeDatalog=true;
        datalogTWB = new DatalogTWB();
        datalogTWB.init(LogName);
    }
    /**
     * TWB method to provide user control of turning the robot.
     * @param deltaAngle turn amount in radians
     */
    public void turn_teleop(double deltaAngle) {
        // Robot Turning:  turn the robot by adjusting the yaw PID setpoint (target)
        setYawTarget(getYawTarget() - deltaAngle );
    }

    /**
     * TWB method translates the robot by setting Position & Pitch Targets.
     *  Recommend using BackNForth_DOE Opmode to determine mmPerLoop and degPerLoop
     * @param forward value from -1 to 1 that is the forward or backward amount
     * @param mmPerLoop robot translation in mm per loop, multiplied by forward
     * @param degPerLoop robot pitch degrees per loop, multiplied by forward
     */
    public void translateDrive(double forward, double mmPerLoop, double degPerLoop) {
        // add small pitch to get it moving. Determine with BackNForth_DOE
        setAutoPitchTarget(forward * degPerLoop);

        // Update posTarget (mm) Note: this value / deltatime = mm per second
        // mmPerLoop of 7 results in a gentle speed
        setPosTarget( getPosTarget() - forward * mmPerLoop );
    }
    public double getDeltaTime() {return deltaTime;}
    public double getPitchTarget() {return pitchTarget;}
    public void setPosTarget(double pos) {posTarget = pos;}
    public double getPos() {return sOdom;}
    public double getPosTarget() {return posTarget;}
    //public void setVeloTarget(double velo) {veloTarget = velo;}
    public double getVeloTarget() {return veloTarget;}
    public double getVelocity() {return linearVelocity;}
    public void setArmPitchTarget (double target) { armPitchTarget = target;   }
    public void setAutoPitchTarget (double target) { autoPitchTarget = target;   }
    public void setYawTarget(double yaw) { yawTarget = yaw; }
    public double getYawTarget() {return yawTarget;}
    public double getPositionVolts() { return positionVolts;}
    public double getPitchVolts() {return pitchVolts;}
    public double getKpitch() {return Kpitch;}
    public double getKpos() {return Kpos;}
    public double getKpitchRate() {return KpitchRate;}
    public double getKvelo() {return Kvelo;}
    public void setKpos(double k) {Kpos = k;}
    public void setKpitch(double k) {Kpitch = k;}
    public void setKpitchRate(double k) {KpitchRate=k;}
    public void setKvelo(double k) {Kvelo = k;}
    public double getYaw() {return yaw; }
    public double getPitch() { return pitch;}
    public double getNewPitch() {
        // get values from the IMU.  Much slower than getPitch.
        orientation = imu.getRobotYawPitchRollAngles();
        pitch = orientation.getPitch(AngleUnit.DEGREES);
        return pitch;
    }
    public double getPitchRate() {return pitchRATE;}
    //public  int getLeftTicks() {return leftTicks;}
    //public  int getRightTicks() {return rightTicks;}
    public double getVerticalCM() {return vertCM;}
    public void setVerticalCM(double verticalCM) {vertCM = verticalCM;}
    //public void setFixedLoopTIme() {fixedLoopTime = true;}
    public void setMMPLoop(double mmpLoop) {MMPLoop = mmpLoop;}
    public void setDEGPLoop(double degpLoop) {DEGPLoop = degpLoop;}
    public double getMMPLoop() {return MMPLoop;}
    public double getDEGPLoop() {return DEGPLoop;}
}