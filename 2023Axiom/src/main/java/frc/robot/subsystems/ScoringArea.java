package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.QOL;


public class ScoringArea{
    static Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);
    static QOL qol2 = new QOL();
    static QOL qol7 = new QOL();
    static QOL qol9 = new QOL();
    static QOL qol11 = new QOL();
    static int column;
    static int row;
    static int cell;
    public ScoringArea(){

    }
    public static String goalkeeper(){
        String[][][] goals = {
            {{"1,1,1","1,1,2","1,1,3"},{"1,2,1","1,2,2","1,2,3"},{"1,3,1","1,3,2","1,3,3"}},
            {{"2,1,1","2,1,2","2,1,3"},{"2,2,1","2,2,2","2,2,3"},{"2,3,1","2,3,2","2,3,3"}},
            {{"3,1,1","3,1,2","3,1,3"},{"3,2,1","3,2,2","3,2,3"},{"3,3,1","3,3,2","3,3,3"}},
            {{"4,1,1","4,1,2","4,1,3"},{"4,2,1","4,2,2","4,2,3"},{"4,3,1","4,3,2","4,3,3"}},
        };
        int column = qol7.incrementjs(7, 0, 3);
        int row = qol9.incrementjs(9, 0, 2);
        int cell = qol11.incrementjs(11, 0, 2);
        System.out.println(goals[column][row][cell]);
        return goals[column][row][cell];
    }
}