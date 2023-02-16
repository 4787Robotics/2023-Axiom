package frc.robot.subsystems;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.QOL;


public class ScoringArea{
    static Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);
    static QOL qol1 = new QOL();
    static QOL qol7 = new QOL();
    static QOL qol9 = new QOL();
    static QOL qol11 = new QOL();

    boolean[][][] goals = {
        {{false, false, false},{false, false, false},{false, false, false}},
        {{false, false, false},{false, false, false},{false, false, false}},
        {{false, false, false},{false, false, false},{false, false, false}},
    };

    public ScoringArea(){

    }
    public boolean goalKeeperVal(){
        int column = qol7.incrementjs(7, 0, 2);
        int row = qol9.incrementjs(9, 0, 2);
        int cell = qol11.incrementjs(11, 0, 2);

        boolean currentGoal = goals[column][row][cell];

        if(joystick.getRawButtonPressed(1)){
            goals[column][row][cell] = !currentGoal;
        }

        return goals[column][row][cell];
    }
    public String goalKeeperPos(){
        int column = qol7.incrementjs(7, 0, 2);
        int row = qol9.incrementjs(9, 0, 2);
        int cell = qol11.incrementjs(11, 0, 2);
        return column+", "+row+", "+cell;
    }
    public void smth(){
        
    }
}