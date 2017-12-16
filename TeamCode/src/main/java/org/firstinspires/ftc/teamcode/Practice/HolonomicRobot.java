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
    int cameraMonitorViewId;

    protected DcMotor leftFront, rightFront, leftBack, rightBack;


    double CurrAngle;
    double CurrAngleOffset;


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

        //motors
        leftFront  = hm.get(DcMotor.class, "leftFront");
        rightFront = hm.get(DcMotor.class, "rightFront");
        leftBack   = hm.get(DcMotor.class, "leftBack");
        rightBack  = hm.get(DcMotor.class, "rightBack");


        //A WHOLE BUNCH of initialization stuff so our motors work fine
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



        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
        VuforiaLocalizer.Parameters Vparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        Vparameters.vuforiaLicenseKey = "Ae/sMsj/////AAAAGeOrJwP8WE9Zj9QPRamQ5yeKrIOrreOb5Ll4kjTkp+iZvq2Qxeku5BJWo7vmmij58qQj4xywefvTAErrY0NnU1QtAvknH55vMIM9BMi3CJ3jQGza2778CKEdZ5Cr7DcGxQQmp0vcO0ndTYfZ8aRwUdnSt88YTn1NjMspDHrwL7ba/7kEG56UQVBNwuQJ9uDf+tE2u1C0peppbLEuj/Fv1cSAaCn4TvE2kaPp/qun3Rzr9K6FPul9WkxA+DG+lPWqgyDS/GtB3UJctkP2L2py0yccc3gFBLsHpNgX9oY3eLxZpvIxCqWHLpgr6NKZUDx/vFWPIUZPYFCBwuy4Hfj9oxL66sOEhyrtWFyxVfuko63v";
        Vparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(Vparameters);


    }










    private VuforiaTrackable relicTemplate;

    public void init()
    {
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTrackables.activate();




        CurrAngle = 0 ;
        CurrAngleOffset = 0;


    }

    //SENSORS ----------------------------------------------------------

    public void updateSensors()
    {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();

        CurrAngle = CurrAngleOffset-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
    public String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }






    public double getAngle()
    {
        return CurrAngle;
    }
    public void setAngleDeg(double degrees){
        CurrAngleOffset = degrees*Math.PI/180;

    }
    public void setAngleRad(double radians)
    {
        CurrAngleOffset = radians;
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





        //actually sets the power to the motors
        leftFront.setPower(LFPower);
        rightFront.setPower(RFPower);
        leftBack.setPower(LBPower);
        rightBack.setPower(RBPower);

    }
    public void intakes(double rightMove, double leftMove, boolean turnRight, boolean turnLeft) {

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
