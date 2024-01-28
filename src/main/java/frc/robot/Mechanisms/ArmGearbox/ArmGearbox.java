// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.Team6328.LoggedTunableNumber;

/** Logger Framework, in here all of the logging and stuff that would affect the beheavior in low level*/
public class ArmGearbox extends SubsystemBase{
    //Interface layers
    private ArmGearboxIO io;
    private ArmGearboxIOInputsAutoLogged inputs = new ArmGearboxIOInputsAutoLogged();

    //TODO: check the spark max burn manager file
    //Dashboard stuff
    /**
     *  private static final Translation2d rootPosition = new Translation2d(0.28, 0.197);
        private Mechanism2d mechanism;
        private MechanismRoot2d mechanismRoot;
        private MechanismLigament2d mechanismLigament;
        https://docs.wpilib.org/en/latest/docs/software/dashboards/glass/mech2d-widget.html
        TODO: Check that out, the mechanism 2d

    */

    //tuneable numbers thru the dashboard
    public static final LoggedTunableNumber armAngle = //Set a "Set Point" angle
        new LoggedTunableNumber("ArmGearbox/armAngle");
    public static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("ArmGearbox/kP");
    public static final LoggedTunableNumber kI = 
        new LoggedTunableNumber("ArmGearbox/kI");
    public static final LoggedTunableNumber kD = 
        new LoggedTunableNumber("ArmGearbox/kD");
    public static final LoggedTunableNumber kIZ = //Value just for Real
        new LoggedTunableNumber("ArmGearbox/kIZ");
    public static final LoggedTunableNumber kFF = 
        new LoggedTunableNumber("ArmGearbox/kFF");
    public static final LoggedTunableNumber kMaxVelo = 
        new LoggedTunableNumber("ArmGearbox/kMaxVelo");
    public static final LoggedTunableNumber kMaxAcc = 
        new LoggedTunableNumber("ArmGearbox/kMaxAcc");


    static{
        switch(Constants.getRobot()){
            case ROBOT_2024:
                //Set the pid values at the very start
                armAngle.initDefault(0);//To use this one create a Tuner Command
                kP.initDefault(0);
                kI.initDefault(0);
                kD.initDefault(0);
                kIZ.initDefault(0);
                kFF.initDefault(0);
                break;
            case ROBOT_SIM:
            armAngle.initDefault(0);
                kP.initDefault(0);
                kI.initDefault(0);
                kD.initDefault(0);
                kIZ.initDefault(0);//Leave it as 0
                kFF.initDefault(0);
                break;
            default:
                break;

        }
    }
    //My java knowledge stops in here, not sure how a static constructor works

    public ArmGearbox(ArmGearboxIO io){
        System.out.println("[Init] Creating ArmGearbox");
        this.io = io;
        io.setBrakeMode(true);

        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        //Update values if they had changed
        if(kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode()) 
        || kIZ.hasChanged(hashCode()) || kFF.hasChanged(hashCode())){
            io.setPIDGains(kP.get(), kI.get(), kD.get(), kIZ.get(), kFF.get());
        }

        if(kMaxAcc.hasChanged(hashCode()) || kMaxVelo.hasChanged(hashCode())){
            io.setSmartMotionGains(kMaxVelo.get(), kMaxAcc.get());
        }
    }
}
