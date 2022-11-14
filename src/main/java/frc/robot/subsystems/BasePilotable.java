// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;



public class BasePilotable extends SubsystemBase {
  
  //Créer l'ensemble des moteurs de la base
  private WPI_TalonFX moteurGauche = new WPI_TalonFX(1);
  private WPI_TalonFX moteurDroit = new WPI_TalonFX(3);
  private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);

  //Créer le gyro
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  
  
 
  private double conversionEncodeur;
 // private final double conversionEncodeur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4);
  private DifferentialDriveOdometry odometry;


  public BasePilotable() {
    //Configure les moteurs
    setRamp(0);
    setBrake(false);
    moteurGauche.setInverted(true);
    moteurDroit.setInverted(false);

    conversionEncodeur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4); 

    //Faire les resets 
    resetEncoder();
    resetGyro();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
    SmartDashboard.putNumber("Angle", getAngle());
    //SmartDashboard.putNumber("Vitesse Droite", getVitesseD());
    //SmartDashboard.putNumber("Vitesse Gauche", getVitesseG());
    SmartDashboard.putNumber("Vitesse Moyenne", getVitesse()); 
    //SmartDashboard.putNumber("Position Droite", getPositionD());
    //SmartDashboard.putNumber("Position Gauche", getPositionG());
    SmartDashboard.putNumber("Position Moyenne", getPosition());
  }

///////MÉTHODE POUR CONDUIRE

  public void conduire(double vx,double vz) {
    //vx la vitesse pour avancer et vz la vitesse pour tourner
    drive.arcadeDrive(-0.8*vx, 0.5*vz);
  }

  public void autoConduire(double voltGauche, double voltDroit) {
    //Fonction conduire pour en Autonomous 
    moteurGauche.setVoltage(voltGauche);
    moteurDroit.setVoltage(voltDroit);
    drive.feed();
  }

  public void stop() {
    //Bein Le Robot Bein y s'arrête
    autoConduire(0, 0);
  }



  /////CONFIG DES MOTEURS
  public void setRamp(double ramp) {
    //Création de la ramp
      moteurDroit.configOpenloopRamp(ramp);
      moteurGauche.configOpenloopRamp(ramp);
    }
    
  public void setBrake(boolean isBrake) {
      //Détermine si le robot brake ou non quand il n'avance pas.
    if (isBrake) {
      moteurDroit.setNeutralMode(NeutralMode.Brake);
      moteurGauche.setNeutralMode(NeutralMode.Brake);
    }
  }



  ////////////////////GYRO
  public double getAngle() {
    //Angle
    return -gyro.getAngle();
  }
  
  public void resetGyro() {
    //Gyro à 0
    gyro.reset();
  }
  
  public double getAngleSpeed() {
    //Vitesse
    return -gyro.getRate();
  } 


  ////////////////ENCODEURS
  
  public double getPositionG() {
    //Position du Moteur Gauche 
    return moteurGauche.getSelectedSensorPosition()*conversionEncodeur;
  }
  
  
  
  public double getPositionD() {
    //Position du Moteur Droit  
    return moteurDroit.getSelectedSensorPosition()*conversionEncodeur;
  }
  
  public double getPosition() {
    //Position Moyenne des Moteurs  
    return (getPositionG() + getPositionD() ) / 2.0;
  }
  
  public double getVitesseD() {
    //Vitesse du Moteur Droit  
    return moteurDroit.getSelectedSensorVelocity()*conversionEncodeur*10;//x10 car les encodeurs des Falcon donne des clics par 100 ms.
  }
  
  public double getVitesseG() {
    //Vitesse du Moteur Gauche  
    return moteurGauche.getSelectedSensorVelocity()*conversionEncodeur*10;
  }
  
  public double getVitesse() {
    //Vitesse du Moteur Moyenne  
    return (getVitesseD() + getVitesseG()) / 2;
  }
  
  
  public void resetEncoder(){
    //Reset Encodeur
    moteurDroit.setSelectedSensorPosition(0);
    moteurGauche.setSelectedSensorPosition(0);
  }
   




  /////////////////CONTRÔLE AVANCÉ
  public double[] getOdometry(){
    double[] position = new double[3];
    double x = getPose().getTranslation().getX();
    double y = getPose().getTranslation().getY();
    double theta = getPose().getRotation().getDegrees();
    position[0] = x;
    position[1] = y;
    position[2] = theta;
    return position;
  }
  
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public void resetOdometry(Pose2d pose){
    //Reset Odometry
    resetEncoder();
    resetGyro();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
    }
  
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
      return new DifferentialDriveWheelSpeeds(getVitesseG(), getVitesseD());
    }

    //Trajectory
  public Trajectory creerTrajectoire(String trajet){
    String trajetJSON = "output/"+trajet+".wpilib.json";
    try{
      var path = Filesystem.getDeployDirectory().toPath().resolve(trajetJSON);
      return TrajectoryUtil.fromPathweaverJson(path);
    }
    catch(IOException e){
      DriverStation.reportError("Unable to open trajectory : " + trajetJSON, e.getStackTrace());
      return null;
    }
    
  }
  public Command ramseteSimple(Trajectory trajectoire){
    //                                                                          
    RamseteCommand ramseteCommand = new RamseteCommand(                         
      trajectoire,                                                              
      this::getPose,                                                            
      new RamseteController(2, 0.7),                                            
      new SimpleMotorFeedforward(Constants.kSRamsete, Constants.kVRamsete, 0),  
      Constants.kinematics,                                                   
      this::getWheelSpeeds,                                              
      new PIDController(Constants.kPRamsete, 0, 0),                          
      new PIDController(Constants.kPRamsete, 0, 0),                                                                                       
      this::autoConduire,                                                    
      this);                                                                  
      return ramseteCommand.andThen(()->stop());                             
  }

}