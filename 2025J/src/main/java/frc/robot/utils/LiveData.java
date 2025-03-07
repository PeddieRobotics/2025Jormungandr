package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiveData {
    private double constant;
    private String name;
    private boolean bool;
    private Field2d data;  
    private String string; 
    

    public LiveData(double c, String n) {
        constant = c;
        name = n;
        SmartDashboard.putNumber(name, constant);
    }

    public LiveData(boolean b, String n){
        bool = b; 
        name = n; 
        SmartDashboard.putBoolean(name, bool); 
    }

    public LiveData(Field2d f, String n){
        data = f; 
        name = n; 
        SmartDashboard.putData(name, data); 
    }

    public LiveData(String s, String n){
        string = s; 
        name = n; 
        SmartDashboard.putString(name, string); 
    }

    public double getNumber() {
        return constant;
    }

    public boolean getBool() { 
        return bool; 
    }

    public String getString(){
        return string; 
    }

    public void setNumber(double c) {
        constant = c;
        SmartDashboard.putNumber(name, constant);
    }

    public void setBoolean(boolean b){
        bool = b; 
        SmartDashboard.putBoolean(name, bool); 
    }

    public void setString(String s){
        string = s; 
        SmartDashboard.putString(name, s); 
    }

    public void setData(Field2d f){
        data = f; 
        SmartDashboard.putData(name, data); 
    }

}