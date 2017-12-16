package org.firstinspires.ftc.teamcode.Practice;


import android.view.OrientationEventListener;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

public class PinchPointRobot extends HolonomicRobot {

    protected DcMotor leftLift, rightLift;

    protected Servo leftGrip, rightGrip;


    private double leftMax = 500, rightMax = 500;

    public PinchPointRobot(HardwareMap map, Telemetry tel)
    {
        super (map, tel);


        leftLift = map.get(DcMotor.class, "leftLift");
        rightLift = map.get(DcMotor.class, "rightLift");

        leftGrip = map.get(Servo.class, "LeftLift");
        rightGrip = map.get(Servo.class, "RightLift");


        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        leftGrip.setDirection(Servo.Direction.FORWARD);
        rightGrip.setDirection(Servo.Direction.REVERSE);


        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }








    //LIFT FUNCTIONS-------------------------------

    public void lift(double power) {

        if (leftLift.getCurrentPosition() > leftMax && power > 0)
            power = 0;
        else if (leftLift.getCurrentPosition() < 0 && power < 0)
            power = 0;
        rightLift.setPower(power);
        leftLift.setPower(power);
    }


    public void grip()
    {
        rightGrip.setPosition(0.72);
        leftGrip.setPosition(0.44);
    }

    public void ungrip()
    {
        rightGrip.setPosition(0.3);
        leftGrip.setPosition(0.16);

    }

    public void gripUp(){
        rightGrip.setPosition(rightGrip.getPosition()+.02);
        leftGrip.setPosition(leftGrip.getPosition()+0.02);
    }
    public void gripDown(){
        rightGrip.setPosition(rightGrip.getPosition()-0.02);
        leftGrip.setPosition(leftGrip.getPosition()-0.02);
    }
    public void liftGrip()
    {
        rightGrip.setPosition(1);
        leftGrip.setPosition(0.74);

    }




}
