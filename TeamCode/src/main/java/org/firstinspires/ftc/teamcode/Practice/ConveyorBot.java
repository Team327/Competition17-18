package org.firstinspires.ftc.teamcode.Practice;


import android.view.OrientationEventListener;

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

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by James on 12/14/2017.
 */

public class ConveyorBot extends HolonomicRobot{

    protected DcMotor leftLift, rightLift, flipper;

    final double flipperMaxPos = 275;


    public ConveyorBot(HardwareMap map, Telemetry tel)
    {

        super(map, tel);
        leftLift = map.get(DcMotor.class, "leftLift");
        rightLift = map.get(DcMotor.class, "rightLift");
        flipper   = map.get(DcMotor.class, "flipper");



        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        flipper.setDirection(DcMotor.Direction.FORWARD);


        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void convey(double power)
    {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }


    public void flip()
    {
        if(flipper.getCurrentPosition() > -flipperMaxPos)
        {
            flipper.setPower(-0.8);
        }
        else
        {
            flipper.setPower(0);
        }

    }
    public void unflip()
    {
        if(flipper.getCurrentPosition() < 0)
        {
            flipper.setPower(0.8);
        }
        else
        {
            flipper.setPower(0);
        }
    }
    public void stopFlip()
    {
        flipper.setPower(0);
    }




}
