// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms.ArmGearbox;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.Team6328.LoggedTunableNumber;

/** Logger Framework, in here all of the logging and stuff that would affect the beheavior in low level*/
public class ArmGearbox extends SubsystemBase{
    //Interface layers
    private ArmGearboxIO io;
    private ArmGearboxIOInputsAutoLogged inputs = new ArmGearboxIOInputsAutoLogged();

    //Dashboard stuff
    /**
     *  private static final Translation2d rootPosition = new Translation2d(0.28, 0.197);
        private Mechanism2d mechanism;
        private MechanismRoot2d mechanismRoot;
        private MechanismLigament2d mechanismLigament;
        https://docs.wpilib.org/en/latest/docs/software/dashboards/glass/mech2d-widget.html
        Check that out
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

    static{
        switch(Constants.getRobot()){
            case ROBOT_2024:
                armAngle.initDefault(0);
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

    public ArmGearbox(){
        
    }

    @Override
    public void periodic() {
    }
}
