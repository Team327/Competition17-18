package org.firstinspires.ftc.teamcode;


        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
        import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Created by James on 10/2/2017.
 */

public class HolonomicRobot {
    OpenGLMatrix lastLocation = null;


    VuforiaLocalizer vuforia;

    protected DcMotor leftFront, rightFront, leftBack, rightBack;


    double CurrAngle;


    //protected Servo

    //protected SensorThings


    public HolonomicRobot(HardwareMap map)
    {
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

    }




    public void init()
    {
        CurrAngle = 270;


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

    public void drive(double xMove, double yMove, double desAngle, Telemetry telemetry)
    {

        //get movement desires
        drive =  Math.sqrt(   Math.pow(yMove, 2) + Math.pow(xMove, 2)  ) ;
        angle =  Math.atan2( yMove , xMove) ;



        turn  =  desAngle;

        coeff = drive;
        if(Math.abs(turn) > coeff) coeff = Math.abs(turn);
        if(coeff > 1) coeff = 1;

        LFPower     =   (drive * Math.sin(angle - Math.PI/4) + turn);
        RFPower     =   (drive * Math.cos(angle - Math.PI/4) - turn);
        LBPower     =   (drive * Math.cos(angle - Math.PI/4) + turn);
        RBPower     =   (drive * Math.sin(angle - Math.PI/4) - turn);



        max = Math.abs(LFPower);
        if (Math.abs(RFPower) > max) max = Math.abs(RFPower);
        if (Math.abs(LBPower) > max) max = Math.abs(LBPower);
        if (Math.abs(RBPower) > max) max = Math.abs(RBPower);
        LFPower /= (max);
        RFPower /= (max);
        LBPower /= (max);
        RBPower /= (max);

        LFPower *= coeff;
        RFPower *= coeff;
        LBPower *= coeff;
        RBPower *= coeff;



        telemetry.addData("coeff:", coeff);
        telemetry.addData("LF:", LFPower);
        telemetry.addData("RF:", RFPower);
        telemetry.addData("LB:", LBPower);
        telemetry.addData("RB:", RBPower);



        leftFront.setPower(LFPower);
        rightFront.setPower(RFPower);
        leftBack.setPower(LBPower);
        rightBack.setPower(RBPower);

    }






}
