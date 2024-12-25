package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPID extends Command{
    private Elevator elevator;
    private double setpoint;
    
    public ElevatorPID(double setpoint){
        this.elevator = Elevator.getSystem();
        this.setpoint = setpoint;
        addRequirements(this.elevator);
    }

    @Override
    public void execute() {
        elevator.pid(this.setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return elevator.stopPID();
    }
}