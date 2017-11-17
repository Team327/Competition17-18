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


    /**creates the adjustable intake for use
     *
     *
     *
     * @param hm    hardware map of the robot
     * @param tel   telemtry module
     */
    public AdjustableIntake(HardwareMap hm, Telemetry tel)
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

    //---------COMPLEX MOVEMENT---------
    public void aimLeft()
    {
        this.setLeftArm(1.0);
        this.setRightArm(.75);
    }

    public void aimRight()
    {
        this.setLeftArm(.75);
        this.setRightArm(1.0);
    }

    public void fullOpen()
    {
        this.setLeftArm(1.0);
        this.setRightArm(1.0);
    }

    public void fullClose()
    {
        this.setLeftArm(.75);
        this.setRightArm(.75);
    }

    public void storeArms()
    {
        this.setLeftArm(0);
        this.setRightArm(0);
    }

    public void intake()
    {
        this.setLeftIntake(1);
        this.setRightIntake(1);
    }

    public void outtake()
    {
        this.setLeftIntake(-1);
        this.setRightIntake(-1);
    }

    public void rotateRight()
    {
        this.setLeftIntake(-1);
        this.setRightIntake(1);
    }

    public void rotateLeft()
    {
        this.setLeftIntake(1);
        this.setRightIntake(-1);
    }


}
