package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableConstant {
    private double constant; 
    private String name; 
    private static ArrayList<TunableConstant> constants = new ArrayList<TunableConstant>(); 

    public TunableConstant(double c, String n){
        constant = c; 
        name = n; 
        constants.add(this); 
        SmartDashboard.putNumber(name, constant);
       
    }

    public void periodic(){
        constant = SmartDashboard.getNumber(name, 0); 
    }

    public double get(){
        return constant; 
    }
    
    public void set(double c){
        constant = c;
        SmartDashboard.putNumber(name, constant);
    }

    public static void updateTunableConstants(){
        for(TunableConstant n : constants){
            n.periodic();
        }
    }
    

}
