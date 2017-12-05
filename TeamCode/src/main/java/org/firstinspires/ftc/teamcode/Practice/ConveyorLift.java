package org.firstinspires.ftc.teamcode.Practice;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Duncan on 12/5/2017.
 */

public class ConveyorLift {
    private double speed;
    //-------Technical--------
    private HardwareMap       hardware;
    private Telemetry         telemetry;


    //-------Motors-----------
    protected DcMotor         leftTrack;
    protected DcMotor         rightTrack;

    public ConveyorLift(HardwareMap har, Telemetry tel){

        hardware   =    har;
        telemetry  =    tel;

        leftTrack  =    hardware.get(DcMotor.class, "LeftTrack");
        rightTrack =    hardware.get(DcMotor.class, "RightTrack");

        rightTrack.setDirection(DcMotor.Direction.REVERSE);
        leftTrack.setDirection(DcMotor.Direction.REVERSE);

        speed = 0;

    }

    public void moveUp(){
        leftTrack.setPower(speed);
        rightTrack.setPower(speed);
    }

    public void moveDown(){
        leftTrack.setPower(-speed);
        rightTrack.setPower(-speed);
    }

    public void increaseSpeed(){
        if(speed<=1){
            speed+=0.2;
        }
    }
    public void decreaseSpeed(){
        if(speed>=1){
            speed-=0.2;
        }
    }

}
