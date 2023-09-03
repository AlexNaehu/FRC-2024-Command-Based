package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private int leftRoller_ID = 25;
    private int rightRoller_ID = 29;

    private final WPI_VictorSPX leftRoller = new WPI_VictorSPX(leftRoller_ID);
    private final WPI_VictorSPX rightRoller = new WPI_VictorSPX(rightRoller_ID);
    
    

    public boolean intakeOpen = true;

    public IntakeSubsystem(){

    }

    public void changeClawState()
    {
        intakeOpen = !intakeOpen;
    }

    public void intake(double power) //takes 2 seconds to close around a cone from full open 
    {
        leftRoller.set(-power*0.8);//0.55
        rightRoller.set(power*0.8);//0.55
        //claw.set(0.5);
    }

    public void output(double power) // takes 2 seconds to open from a cone closed position back to full open
    {
        leftRoller.set(power*0.6);
        rightRoller.set(-power*0.6);
        //claw.set(-0.5);
    }

    public boolean isIntakeOpen()
    {
        return intakeOpen;
    }

    public double getLeftRollerPower()
    {
        return leftRoller.get();
        //return claw.get();
    }

    public double getRightRollerPower()
    {
        return rightRoller.get();
    }

}
