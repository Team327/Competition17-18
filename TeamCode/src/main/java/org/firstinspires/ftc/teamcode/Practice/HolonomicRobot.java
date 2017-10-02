package org.firstinspires.ftc.teamcode.Practice;


        import android.view.OrientationEventListener;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.Func;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.Position;
        import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
        import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
        import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

        import java.util.Locale;


/**
 * Created by James on 10/2/2017.
 */

public class HolonomicRobot {
    Telemetry telemetry;
    OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;

    protected DcMotor leftFront, rightFront, leftBack, rightBack;


    double CurrAngle;


    //protected Servo

    //protected SensorThings
    protected BNO055IMU imu;
        protected Orientation angles;
        protected Acceleration gravity;


    public HolonomicRobot(HardwareMap map, Telemetry tel)
    {
        telemetry = tel;
        hardwareInit(map);
        init();

    }


    public void hardwareInit(HardwareMap hm)
    {
        this.telemetry = telemetry;
        //motors
        leftFront  = hm.get(DcMotor.class, "leftFront");
        rightFront = hm.get(DcMotor.class, "rightFront");
        leftBack   = hm.get(DcMotor.class, "leftBack");
        rightBack  = hm.get(DcMotor.class, "rightBack");


        //A WHOLE BUMCH of initialization stuff so out motors work fine
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //servos

        //sensors

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hm.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();


        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


    }


    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatRadians(AngleUnit.RADIANS.fromUnit(angleUnit, angle));
    }

    String formatRadians(double radians){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.RADIANS.normalize(radians));
    }







    public void init()
    {
        CurrAngle = 270;


    }

    //SENSORS ----------------------------------------------------------

    public void updateSensors()
    {
        telemetry.update();
    }


    //DRIVETRAIN -------------------------------------------------------

    private double LFPower;
    private double RFPower;
    private double LBPower;
    private double RBPower;
    private double max;
    private double drive;
    private double angle;
    private double turn;
    private double coeff;


    /** Intakes stick_x, stick_y and right stick_x to move the robot according to the holonomic equation
     *
     * @param xMove: left_stick_x value, or the amount to move left/right
     * @param yMove: left_stick_y value, or the amount to move up/down
     * @param turnForce: right_stick_x, or the amount to turn
     */
    public void drive(double xMove, double yMove, double turnForce)
    {

        //get desired movement magnitude
        drive =  Math.sqrt(   Math.pow(yMove, 2) + Math.pow(xMove, 2)  ) ;

        //get desired movement direction FIELD ORIENTED by CurrAngle
        angle =  Math.atan2( yMove , xMove) - CurrAngle;

        //Rotation based turning
        turn = -Math.abs(turnForce) * turnForce;

        //Place to put the desired angle input


        //determines drive coefficient based on turn force and drive force
        coeff = drive;
        if(Math.abs(turn) > coeff) coeff = Math.abs(turn);
        if(coeff > 1) coeff = 1;


        //sets each motor to the power of a trig function
        //Axis is rotated CW PI/4
        LFPower     =   (drive * Math.sin(angle - Math.PI/4) + turn);
        RFPower     =   (drive * Math.cos(angle - Math.PI/4) - turn);
        LBPower     =   (drive * Math.cos(angle - Math.PI/4) + turn);
        RBPower     =   (drive * Math.sin(angle - Math.PI/4) - turn);


        //finds maximum power and makes that 1, increases all others proportionally
        max = Math.abs(LFPower);
        if (Math.abs(RFPower) > max) max = Math.abs(RFPower);
        if (Math.abs(LBPower) > max) max = Math.abs(LBPower);
        if (Math.abs(RBPower) > max) max = Math.abs(RBPower);
        LFPower /= (max);
        RFPower /= (max);
        LBPower /= (max);
        RBPower /= (max);


        //multiplies by driving coefficient so that sensitivity exists
        LFPower *= coeff;
        RFPower *= coeff;
        LBPower *= coeff;
        RBPower *= coeff;


        //debug telemetry
        telemetry.addData("coeff:", coeff   );
        telemetry.addData("LF:",    LFPower );
        telemetry.addData("RF:",    RFPower );
        telemetry.addData("LB:",    LBPower );
        telemetry.addData("RB:",    RBPower );


        //actually sets the power to the motors
        leftFront.setPower(LFPower);
        rightFront.setPower(RFPower);
        leftBack.setPower(LBPower);
        rightBack.setPower(RBPower);

    }

    /* uncomment if you want to try out desAngle stuff
    private double pid(double state, double desState, double kp, double kd, double ki)
    {
        double error = desState - state;

        double p = error * kp;






        return p+i+d;
    }
    */







}
