package org.firstinspires.ftc.teamcode.Practice;

/**
 * Created by James on 1/26/2018.
 */

public class Auto1 {
    private     HolonomicRobot  robot;
    private     enum            STATE
    {
        START,
        DOWNARM,    DETECTCOLOR,    TURN1,  UPARM,  TURN2,
        //DETECTSIDE, MOVETOCOL,      MOVEUP, PLACE,  RETURN,
        END

    };


    private     STATE   currState;

    public Auto1(HolonomicRobot rob)
    {
        robot = rob;
        currState = STATE.START;

    }



    public void loop()
    {
        switch(currState)
        {

            case START:

                break;

            case DOWNARM:

                break;

            case DETECTCOLOR:

                break;

            case TURN1:

                break;

            case UPARM:

                break;

            case TURN2:

                break;

            case END:

                break;

            default:

                break;

        }
    }
}
