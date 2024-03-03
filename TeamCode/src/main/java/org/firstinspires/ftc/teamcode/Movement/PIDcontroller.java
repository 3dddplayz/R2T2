package org.firstinspires.ftc.teamcode.Movement;

public class PIDcontroller {
    public double Pcontroller;
    public double Icontroller;
    public double Dcontroller;
    public double Integral;
    public int ITime;
    public double LastError;
    public double Error;
    public PIDcontroller(double P, double I, double D, int ILength){
        Pcontroller = P;
        Icontroller = I;
        Dcontroller = D;
        Integral = 0;
        ITime = ILength;
    }
    public double update(double target, double position){
        LastError = Error;
        Error = target - position;
        if(Math.abs(Error+360)<Math.abs(Error)){
            Error = Error+360;
        } else if(Math.abs(Error-360)<Math.abs(Error)){
            Error = Error-360;
        }

        double finalCorrection = Error*Pcontroller;

        Integral = (Integral*(ITime-1) + finalCorrection) / ITime;
        finalCorrection += Integral * Icontroller;

        finalCorrection += (Error - LastError) * Dcontroller;
        return finalCorrection;
    }
}