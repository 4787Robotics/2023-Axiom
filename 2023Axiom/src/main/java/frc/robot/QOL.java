package frc.robot;
// ill put some type convertion stuff in here sometime
public class QOL {
    public static boolean Switch = false;

    QOL() {

    }
    public static boolean toggle(boolean button1, boolean button2){
        
        if (button1){
            if(button2 && !Switch){
                Switch = true;
            }
            else if(button2 && Switch){
                Switch = false;
            }
            
        }  
        return Switch;
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

