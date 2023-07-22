package frc.robot.subsystems;

import frc.robot.CJoystick;
import frc.robot.QOL;


public class ScoringArea{
    static CJoystick joystick = new CJoystick();
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

        if(joystick.joystickButton1Down()){
            goals[column][row][cell] = !goals[column][row][cell];
        }

        return goals[column][row][cell];
    }
    public String goalKeeperPos(){
        int column = qol7.incrementjs(7, 0, 2);
        int row = qol9.incrementjs(9, 0, 2);
        int cell = qol11.incrementjs(11, 0, 2);
        return column+", "+row+", "+cell;
    }
    public Boolean goal(int x, int y, int z){
        return goals[x][y][z];
    }
}