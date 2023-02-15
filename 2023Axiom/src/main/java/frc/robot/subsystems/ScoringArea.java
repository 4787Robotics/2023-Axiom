package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.CJoystick;
import frc.robot.Constants;
import frc.robot.QOL;


public class ScoringArea{
    static Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);
    static QOL qol1 = new QOL();
    static QOL qol7 = new QOL();
    static QOL qol9 = new QOL();
    static QOL qol11 = new QOL();
    static boolean initst;
    public ScoringArea(){

    }
    public static boolean goalkeeper(){
        boolean[][][] goals = {
            {{initst,initst,initst},{initst,initst,initst},{initst,initst,initst}},
            {{initst,initst,initst},{initst,initst,initst},{initst,initst,initst}},
            {{initst,initst,initst},{initst,initst,initst},{initst,initst,initst}},
            {{initst,initst,initst},{initst,initst,initst},{initst,initst,initst}},
        };

        int column = qol7.incrementjs(7, 0, 3);
        int row = qol9.incrementjs(9, 0, 2);
        int cell = qol11.incrementjs(11, 0, 2);


        if(joystick.getRawButtonPressed(1)) {
            goals[column][row][cell] = !goals[column][row][cell];
        }
        System.out.println(goals[column][row][cell]);
        return goals[column][row][cell];
    }
}