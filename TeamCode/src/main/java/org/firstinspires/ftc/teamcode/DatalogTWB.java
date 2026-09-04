package org.firstinspires.ftc.teamcode;

/**
 * DatalogTWB is a class for providing datalogging for the Two Wheeled Balancing Bot
 *  where the data is to be written every loop cycle.
 */
public class DatalogTWB {

    private DatalogTWBMain datalog; // create the data logger object

    /**
     * TWB init. Called once at initialization
     */
    public void init(String fileName) {
        datalog = new DatalogTWBMain(fileName);
    }

    public void logPosPitch(double pos, double posX, double posY,
                            double posTarget, double velocity, double acceleration,
                            double pitch, double pitchTarget, double pitchRATE,
                            double yaw,  double yawTarget, double posVolts,
                            double pitchVolts, double dt) {
        // Data log
        // Note that the order in which we set datalog fields
        // does *not* matter! Order is configured inside the Datalog class constructor.
        datalog.pos.set(pos);
        datalog.posX.set(posX);
        datalog.posY.set(posY);
        datalog.posTarget.set(posTarget);
        datalog.acceleration.set(acceleration);
        datalog.pitch.set(pitch);
        datalog.pitchTarget.set(pitchTarget);
        datalog.pitchRATE.set(pitchRATE);
        datalog.yaw.set(yaw);
        datalog.yawTarget.set(yawTarget);
        datalog.linVelo.set(velocity);
        datalog.positionVolts.set(posVolts); // look for saturation when tuning
        datalog.pitchVolts.set(pitchVolts);  // look for saturation when tuning
        datalog.totalVolts.set(posVolts+pitchVolts);
        datalog.dt.set(dt);
    }
    public void writeLineTWB() {
        // The logged timestamp is taken when writeLine() is called.
        datalog.writeLine();
    }
    /**
     * Datalog class encapsulates all the fields that will go into the datalog.
     */
    public static class DatalogTWBMain {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField pitch = new Datalogger.GenericField("Pitch");
        public Datalogger.GenericField pitchTarget = new Datalogger.GenericField("PitchTarget");
        public Datalogger.GenericField pitchRATE = new Datalogger.GenericField("pitchRATE");
        public Datalogger.GenericField pos = new Datalogger.GenericField("Pos_MyOdo");
        public Datalogger.GenericField posX = new Datalogger.GenericField("X");
        public Datalogger.GenericField posY = new Datalogger.GenericField("Y");
        public Datalogger.GenericField posTarget = new Datalogger.GenericField("PosTarget");
        public Datalogger.GenericField acceleration = new Datalogger.GenericField("Acceleration");
        public Datalogger.GenericField yaw = new Datalogger.GenericField("Yaw");
        public Datalogger.GenericField yawTarget = new Datalogger.GenericField("yawTarget");
        public Datalogger.GenericField linVelo = new Datalogger.GenericField("linearVelo");
        public Datalogger.GenericField positionVolts = new Datalogger.GenericField("positionVolts");
        public Datalogger.GenericField pitchVolts = new Datalogger.GenericField("pitchVolts");
        public Datalogger.GenericField totalVolts = new Datalogger.GenericField("TotalVots");
        public Datalogger.GenericField dt = new Datalogger.GenericField("DeltaTime");

        public DatalogTWBMain(String name) {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            pitch,
                            pitchTarget,
                            pitchRATE,
                            pos,
                            posX,
                            posY,
                            posTarget,
                            acceleration,
                            yaw,
                            yawTarget,
                            linVelo,
                            positionVolts,
                            pitchVolts,
                            totalVolts,
                            dt
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine() {
            datalogger.writeLine();
        }
    }

}
