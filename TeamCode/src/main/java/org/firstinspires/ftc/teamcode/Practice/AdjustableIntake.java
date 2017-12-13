package org.firstinspires.ftc.teamcode.Practice;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by James on 11/16/2017.
 */

public class AdjustableIntake {

    //-----Technical stuff-----
    private HardwareMap     hardware;
    private Telemetry       telemetry;


    //------SERVOS-----
    //arms of the intake
    private Servo       leftArm;
    private Servo       rightArm;

    //actual continuous servos that pull blocks in
    private CRServo     leftIntake;
    private CRServo     rightIntake;




    //Servo positions, set in the constructor
    private double leftStow;
    private double leftIn;
    private double leftOut;
    private double rightStow;
    private double rightIn;
    private double rightOut;





    private double rightPos;
    private double leftPos;



    /**creates the adjustable intake for use
     *
     *
     *
     * @param hm    hardware map of the robot
     * @param tel   telemtry module
     */
    public AdjustableIntake(HardwareMap hm, Telemetry tel, double lStow, double lIn, double lOut, double rStow, double rIn, double rOut)
    {
        hardware    =   hm;
        telemetry   =   tel;

        leftArm     =       hardware.get(Servo.class, "LeftArm");
        rightArm    =       hardware.get(Servo.class, "RightArm");
        leftIntake  =       hardware.get(CRServo.class, "LeftIntake");
        rightIntake =       hardware.get(CRServo.class, "RightIntake");

        //reverse these guys so that they can act the same as the left ones
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftStow = lStow;
        leftIn = lIn;
        leftOut = lOut;

        rightStow = rStow;
        rightIn = rIn;
        rightOut = rOut;


        rightPos = rightStow;
        leftPos  = leftStow;



    }

    //-----BASIC MOVEMENT-------
    public void setLeftArm(double angle)
    {
        leftArm.setPosition(angle);
    }
    public void setRightArm(double angle)
    {
        rightArm.setPosition(angle);
    }
    public void setLeftIntake(double pow)
    {
        leftIntake.setPower(pow);
    }
    public void setRightIntake(double pow)
    {
        rightIntake.setPower(pow);
    }

    //-------BASIC SETTINGS--------
    public void setArmScale(double leftMin, double leftMax, double rightMin, double rightMax)
    {
        leftArm.scaleRange(leftMin, leftMax);
        rightArm.scaleRange(rightMin, rightMax);
    }



    //---------ADJUSTMENT-----------
    public void shiftLeft()
    {
        leftPos -= 0.01;
        rightPos += 0.01;

        if(rightPos > rightOut) rightPos = rightOut;
        if(leftPos < leftIn) leftPos = leftIn;


        this.setLeftArm(leftPos);
        this.setRightArm(rightPos);
    }


    public void shiftRight()
    {
        leftPos += 0.01;
        rightPos -= 0.01;

        if(rightPos < rightIn) rightPos = rightIn;
        if(leftPos > leftOut) leftPos = leftOut;


        this.setLeftArm(leftPos);
        this.setRightArm(rightPos);
    }



    //---------COMPLEX MOVEMENT---------
    public void aimLeft()
    {
        this.setLeftArm(leftOut);
        this.setRightArm(rightIn);
    }

    public void aimRight()
    {
        this.setLeftArm(leftIn);
        this.setRightArm(rightOut);
    }

    public void fullOpen()
    {
        this.setLeftArm(leftOut);
        this.setRightArm(rightOut);
    }

    public void fullClose()
    {
        this.setLeftArm(leftIn);
        this.setRightArm(rightIn);
    }

    public void storeArms()
    {
        this.setLeftArm(leftStow);
        this.setRightArm(rightStow);
    }

    public void intake()
    {
        this.setLeftIntake(-1);
        this.setRightIntake(-1);
    }

    public void outtake()
    {
        this.setLeftIntake(1);
        this.setRightIntake(1);
    }

    public void rotateRight()
    {
        this.setLeftIntake(1);
        this.setRightIntake(-1);
    }

    public void rotateLeft()
    {
        this.setLeftIntake(-1);
        this.setRightIntake(1);
    }

    public void stopIntake()
    {
        this.setLeftIntake(0);
        this.setRightIntake(0);
    }


}
