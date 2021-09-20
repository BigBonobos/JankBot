//region_Copyright

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//endregion

package frc.robot;

import java.sql.Time;

//navx imports
import com.kauailabs.navx.frc.*;
//spark max/neos imports
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//region_Imports

//regular imports
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

  //endregion

public class Robot extends TimedRobot {

  //region_Variables

    //joysticks
      public Joystick j_Left = new Joystick(0);
      public Joystick j_Right = new Joystick(1);
      public Joystick j_Operator = new Joystick(2);
      public XboxController j_XboxController = new XboxController(4);

    //neos
      public CANSparkMax m_Left1 = new CANSparkMax(42, MotorType.kBrushless); //OG 12
      public CANSparkMax m_Right1 = new CANSparkMax(61, MotorType.kBrushless); //OG 1
    

    //neo encoders
      public CANEncoder e_Left1 = m_Left1.getEncoder(); //positive forward for Left
      public CANEncoder e_Right1 = m_Right1.getEncoder(); //negative forward for right

    //public Timer timer;

    //neo pidcontrollers
      public CANPIDController pc_Left1 = m_Left1.getPIDController();
      public CANPIDController pc_Right1 = m_Right1.getPIDController();


    //neo controllers
      public SpeedControllerGroup m_Left = new SpeedControllerGroup(m_Left1);
      public SpeedControllerGroup m_Right = new SpeedControllerGroup(m_Right1);
      public DifferentialDrive m_DriveTrain = new DifferentialDrive(m_Left, m_Right); //negative power makes bot move forward, positive power makes bot move packwards

    //tuning variables
      public double kP_Left1, kI_Left1, kD_Left1, kIz_Left1, kFF_Left1;
      public double kP_Right1, kI_Right1, kD_Right1, kIz_Right1, kFF_Right1;

    //solenoid variables
      //public Compressor c = new Compressor(0);

    //navx variables
      public AHRS navX = new AHRS(SPI.Port.kMXP);
      public float imu_Yaw;
      public boolean resetYaw = false;

    //vision variables
      //public NetworkTableEntry xEntry;
    //sensors
    //logic variables
      //gear switching
        //public boolean lowGear=true;
        //public boolean switchGears;
      //intake booleans
      //ball counting variables
      //shooting booleans
      //controlpanel variables
      //gamedata
      //climb variables
      //variables for auto phase
  //endregion
 
  @Override
  public void robotInit() {
    m_Left.setInverted(true);
    m_Right.setInverted(false);


    //region_SettingPidVariables
      kP_Left1 = .0001;
      kI_Left1 = 0;
      kD_Left1 = 0.01;
      kIz_Left1 = 0;
      kFF_Left1 = .0001746724891;

      kP_Right1 = .0001;
      kI_Right1 = 0;
      kD_Right1 = 0.01;
      kIz_Right1 = 0;
      kFF_Right1 = .0001746724891;
      //endregion

    //region_SettingPidValues
      pc_Left1.setP(kP_Left1);
      pc_Left1.setI(kI_Left1);
      pc_Left1.setD(kD_Left1);
      pc_Left1.setIZone(kIz_Left1);
      pc_Left1.setFF(kFF_Left1);

      pc_Right1.setP(kP_Right1);
      pc_Right1.setI(kI_Right1);
      pc_Right1.setD(kD_Right1);
      pc_Right1.setIZone(kIz_Right1);
      pc_Right1.setFF(kFF_Right1);
    //endregion

  }

  @Override
  public void autonomousInit() {
    e_Right1.setPosition(0);
    e_Left1.setPosition(0);
 
  }

  @Override
  public void autonomousPeriodic() {
    //SmartDashboard.putNumber("right encoder 1 ", e_Right1.getPosition());
    //SmartDashboard.putNumber("right encoder 2 ", e_Right2.getPosition());
    //SmartDashboard.putNumber("left encoder 1 ", e_Left1.getPosition());
    //SmartDashboard.putNumber("left encoder 2 ", e_Left2.getPosition());
    }

    // switch (autoCase){
    //   case 1:
    //     Timer.delay(2);
    //     autoCase++;
    //     break;
    //   case 2:
    //     driveStraight(1, 1000);


    //}

  @Override
  public void teleopInit() {
    e_Right1.setPosition(0);
    e_Left1.setPosition(0);
    m_DriveTrain.stopMotor();
  }

  @Override
  public void teleopPeriodic() {
    //gettingVision();
    joystickControl();
    /*//if/else series controlling drivetrain motors
    if (j_Right.getTrigger()){
      visionTracking();
    }
    else {
      joystickControl();
      gearSwitching();
    }*/
  


    //if/else series controlling intaking and shooting balls
    /*if (j_Operator.getRawButton(1)){
      intakingBalls();
      oldBallBoolean = newBallBoolean;
    }
    else if (j_Operator.getRawButton(2)) {
      shootingBalls();
    }
    else {
      intake();
      readyToFeed = false;
      m_BotShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_TopShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);
      m_TopShooter.stopMotor();
      m_BotShooter.stopMotor();
    } */

    /*if(j_Operator.getRawButton(7)){
      if(targetColor == 0){
        controlPanelRevolution();
      }
      else{
        controlPanelColorSpin();
      }
    }*/

    //climb();
    //gameData();
    //controlPanelExtend();
    //tiltingControl();
    //ballCounterReset();
    //lidarDistance();
    //colorFinder();

    //region_SmartDashboard
      SmartDashboard.putNumber("Right encoders", e_Right1.getPosition());
      SmartDashboard.putNumber("Left encoders", e_Left1.getPosition());

    //endregion
  }

  @Override
  public void testInit() {
    navX.zeroYaw();
    e_Left1.setPosition(0);
    e_Right1.setPosition(0);
    //driveBack(1, 500);
  
  }


  @Override
  public void testPeriodic() {
  }

  //region_Methods

    public void joystickControl(){ //method for implementing our lowgear/highgear modes into our driver controls
        m_DriveTrain.tankDrive(j_Left.getY() * .7, -j_Right.getY() * .7);
        //m_DriveTrain.tankDrive(j_XboxController.getY(Hand.kLeft)*.75, -j_XboxController.getY(Hand.kRight)*.75);
        //m_DriveTrain.tankDrive(j_XboxController.getY(Hand.kLeft), -j_XboxController.getY(Hand.kRight));
    }

	public void driveStraight(double feet, double speed){

      //change encoder distance to feet (I didn't write this lmao -R)
      double encoderFeet = feet * 6.095233693;

      //flip speed if negative
      if (feet < 0) {
        speed = -speed;
      }

      //check by absolute value to make sure pos is changing the right distance
      if(Math.abs(e_Left1.getPosition()) < encoderFeet || Math.abs(e_Right1.getPosition()) < encoderFeet){
        
        // left needs to be opposite of right
        pc_Left1.setReference(speed, ControlType.kVelocity);
        pc_Right1.setReference(-speed, ControlType.kVelocity);
      }
      else{
        //stop the motors and reset the encoder counts for the following methods
        m_DriveTrain.stopMotor();
        e_Right1.setPosition(0);
        e_Left1.setPosition(0);
        //increment upward

      }
    }
  
    public void driveBack(double feet, double speed){

      //change encoder distance to feet (I didn't write this lmao -R)
      double encoderFeet = feet * 6.095233693;

      //flip speed if negative
      // if (feet < 0) {
      //   speed = -speed;
      // }

      //check by absolute value to make sure pos is changing the right distance
      if(Math.abs(e_Left1.getPosition()) < Math.abs(encoderFeet) || Math.abs(e_Right1.getPosition()) < Math.abs(encoderFeet)){
        
        // left needs to be opposite of right
        pc_Left1.setReference(-speed, ControlType.kVelocity);
        pc_Right1.setReference(speed, ControlType.kVelocity);
      }
      else{

        //stop the motors and reset the encoder counts for the following methods
        m_DriveTrain.stopMotor();
        e_Right1.setPosition(0);
        e_Left1.setPosition(0);
        //increment upward

      }
    }
    public void rightTurn(double targetAngle){
      if(resetYaw == false){
        navX.zeroYaw();
        resetYaw = true;
      }
      double actualYaw = Math.abs(navX.getYaw() % 360);
      if (Math.abs(actualYaw - targetAngle) < 8){
        pc_Left1.setReference(0, ControlType.kVelocity);
        pc_Right1.setReference(0, ControlType.kVelocity);
        e_Right1.setPosition(0);
        e_Left1.setPosition(0);
        navX.reset();
      }
      else{
        pc_Left1.setReference(1000, ControlType.kVelocity);
        pc_Right1.setReference(1000, ControlType.kVelocity);

      }
    }
    public void leftTurn(double targetAngle){
      if(resetYaw == false){
        navX.zeroYaw();
        resetYaw = true;
      }
      double actualYaw = Math.abs(navX.getYaw() % 360);
      if (Math.abs(actualYaw - targetAngle) < 6){
        pc_Left1.setReference(0, ControlType.kVelocity);
        pc_Right1.setReference(0, ControlType.kVelocity);
        e_Right1.setPosition(0);
        e_Left1.setPosition(0);
        resetYaw = false;
      }
      else{
        pc_Left1.setReference(-1000, ControlType.kVelocity);
        pc_Right1.setReference(-1000, ControlType.kVelocity);
      }
    }
    
    //end region
}
