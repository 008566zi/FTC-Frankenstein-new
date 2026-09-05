public abstract class BaseClass extends LinearOpMode {

    // Globally Declared Sensors
    public IMU gyro;

    // Module Classes
    public Drive driveModule = null; // This is an actual class with various methods

    // Global Variables
    public int exampleVariable = 0;

    // Initialize Hardware Function
    public void initHardware() throws InterruptedException {
        // Hubs
        List<LynxModule> allHubs;
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motors
        /* DcMotor armMotor = hardwareMap.get(DcMotor.class, "Drive Motor");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); */

        // Drivetrain
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class), "Drive Motor");
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class), "Drive Motor");
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class), "Drive Motor");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class), "Drive Motor");
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Init Module class
        //armModule = new Arm(armMotor);

        //Drivetrain
        driveModule = new Drive(backLeftMotor,backRightMotor,frontLeftMotor,frontRightMotor);
    }

    //Utility Functions
    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // Allows you to connect opModes to the base class
    @Override
    public abstract void runOpMode() throws InterruptedException;
}