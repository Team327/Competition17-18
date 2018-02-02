package org.firstinspires.ftc.teamcode.Practice;

/**
 * Created by James on 1/26/2018.
 */

public class Auto1 {
    private     ConveyorBot  robot;
    private     enum            STATE
    {
        START,
        DOWNARM,    DETECTCOLOR,    TURNLEFT1, TURNLEFT2, TURNRIGHT1, TURNRIGHT2,
        //DETECTSIDE, MOVETOCOL,      MOVEUP, PLACE,  RETURN,
        END

    };



    private     double  startTime;
    private     boolean AllIsRed;
    private     STATE   currState;

    public Auto1(ConveyorBot rob)
    {
        AllIsRed = true;
        robot = rob;
        currState = STATE.START;

    }


    public void startTime()
    {
        startTime = System.currentTimeMillis();
    }

    public void loop()
    {
        double elapsed = System.currentTimeMillis() - startTime;
        switch(currState)
        {

            case START:
                robot.drive(0,0,-.3);
                if(elapsed > 500) currState = STATE.DOWNARM;
                break;

            case DOWNARM:
                robot.drive(0,0,0);
                robot.dropJewelArm();
                if(elapsed > 1500) currState = STATE.DETECTCOLOR;
                break;

            case DETECTCOLOR:
                robot.drive(0,0,0);

                if(robot.jewelIsRed() == AllIsRed) currState = STATE.TURNRIGHT1;
                else                               currState = STATE.TURNLEFT1;
                break;

            case TURNRIGHT1:
                robot.drive(0,0,0.3);
                if (elapsed > 2000) currState = STATE.TURNRIGHT2;
                break;

            case TURNRIGHT2:
                robot.liftJewelArm();
                robot.drive(0,0,-0.3);
                if (elapsed > 2500) currState = STATE.END;
                break;

            case TURNLEFT1:
                robot.drive(0,0,-0.3);
                if (elapsed > 2000) currState = STATE.TURNRIGHT2;
                break;

            case TURNLEFT2:
                robot.liftJewelArm();
                robot.drive(0,0,0.3);
                if (elapsed > 2500) currState = STATE.END;
                break;

            case END:
                robot.drive(0,0,0);
                break;

            default:

                break;

        }
    }
}
