package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

// ill put some type convertion stuff in here sometime
public class QOL {
    boolean Switch = false;
    int increment;
    int check = 0;
    Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);
    QOL() {
        
    }
    public boolean togglejs(int button){
        if(joystick.getRawButtonPressed(button)){
            Switch = !Switch;
        }
        return Switch;
    }
    public int incrementjs(int button,int start, int max){
        if(check == 0){
            increment = start;
            check = 1;
        }
        if(joystick.getRawButtonPressed(button)){
            increment++;
            if(increment > max){
                increment = start;
            }
        }
        return increment;
    }
    //turns 1 or 0 into true or false, respectively
    public boolean toBool(int arg){
        if(arg == 1){
            return true;
        }
        else{
            return false;
        }
    }

    public int toInt(double arg){
        return (int) Math.round(arg);
    }
}

