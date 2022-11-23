// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class BasePilotable extends SubsystemBase {
  
  private WPI_TalonFX moteurGauche = new WPI_TalonFX(1);
  private WPI_TalonFX moteurDroit = new WPI_TalonFX(3);
  

  TalonFXSimCollection m_leftDriveSim = moteurGauche.getSimCollection();
  TalonFXSimCollection m_rightDriveSim = moteurDroit.getSimCollection();

  private DifferentialDrive drive = new DifferentialDrive(moteurGauche, moteurDroit);

  private double conversionEncodeur;
  private final double conversionMoteur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4);
  //Initialisation du gyro
  private ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  
// Create the simulated gyro object, used for setting the gyro
// angle. Like EncoderSim, this does not need to be commented out
// when deploying code to the roboRIO.
private ADXRS450_GyroSim m_gyroSim = new ADXRS450_GyroSim(gyro);

  private MedianFilter filter = new MedianFilter(5);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  private SimpleMotorFeedforward tournerFF = new SimpleMotorFeedforward(0.496, 0.0287);
  private ProfiledPIDController tournerPID = new ProfiledPIDController(0.20, 0, 0, new TrapezoidProfile.Constraints(90, 90));
  
  public BasePilotable() {
    //Configure les moteurs
    conversionEncodeur = (1.0/2048)*(14.0/72)*(16.0/44)*Math.PI*Units.inchesToMeters(4); 
    setRamp(0);
    setBrake(false);
    moteurGauche.setInverted(true);
    moteurDroit.setInverted(false);
    SmartDashboard.putData("Field", m_field);

  }

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

  @Override
  public void periodic() {
      // This will get the simulated sensor readings that we set
  // in the previous article while in simulation, but will use
  // real values on the robot itself.
   odometry.update(gyro.getRotation2d(),
            nativeUnitsToDistanceMeters(moteurGauche.getSelectedSensorPosition()),
            nativeUnitsToDistanceMeters(moteurDroit.getSelectedSensorPosition()));
  
      m_field.setRobotPose(odometry.getPoseMeters());
  }

  public double getAngle() {
    //Angle à Implémenter dans le DashBoard
    return -gyro.getAngle();
  }
  
  public void resetGyro() {
    //Gyro à 0
    gyro.reset();
  }
  
  public double getAngleSpeed() {
    //Vitesse de l'angle à Implémenter dans le DashBoard
    return filter.calculate(-gyro.getRate());
  } 
  
  public double getPositionG() {
    //Position du Moteur Gauche à Implémenter dans le DashBoard
    return moteurGauche.getSelectedSensorPosition()*conversionMoteur;
  }
  
  
  
  public double getPositionD() {
    //Position du Moteur Droit à Implémenter dans le DashBoard
    return moteurDroit.getSelectedSensorPosition()*conversionMoteur;
  }
  
  public double getPosition() {
    //Position Moyenne des Moteurs à Implémenter dans le DashBoard
    return (getPositionG() + getPositionD() ) / 2.0;
  }
  
  public double getVitesseD() {
    //Vitesse du Moteur Droit à Implémenter dans le DashBoard
    return moteurDroit.getSelectedSensorVelocity()*conversionEncodeur*10;//x10 car les encodeurs des Falcon donne des clics par 100 ms.
  }
  
  public double getVitesseG() {
    //Vitesse du Moteur Gauche à Implémenter dans le DashBoard
    return moteurGauche.getSelectedSensorVelocity()*conversionEncodeur*10;
  }
  
  public double getVitesse() {
    //Vitesse du Moteur Moyenne à Implémenter dans le DashBoard
    return (getVitesseD() + getVitesseG()) / 2;
  }
  
  
  public void resetEncoder(){
    //Reset Encodeur
    moteurDroit.setSelectedSensorPosition(0);
    moteurGauche.setSelectedSensorPosition(0);
  }
   
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

  public double getVoltagePIDF(double angleCible, DoubleSupplier mesure){
    return tournerPID.calculate(mesure.getAsDouble(), angleCible) + tournerFF.calculate(tournerPID.getSetpoint().velocity);
  }

  public boolean atAngleCible(){
   return tournerPID.atGoal();
  }

  /*SIMULATION */
  final int kCountsPerRev = 4096;  //Encoder counts per revolution of the motor shaft.
  final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
  final double kGearRatio = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
  final double kWheelRadiusInches = 3;
  final int k100msPerSecond = 10;

  /* Simulation model of the drivetrain */
  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2),  //2 Falcon 500s on each side of the drivetrain.
    conversionMoteur,               //Standard AndyMark Gearing reduction.
    2.1,                      //MOI of 2.1 kg m^2 (from CAD model).
    26.5,                     //Mass of the robot is 26.5 kg.
    Units.inchesToMeters(kWheelRadiusInches),  //Robot uses 3" radius (6" diameter) wheels.
    0.546,                    //Distance between wheels is _ meters.
    
    /*
     * The standard deviations for measurement noise:
     * x and y:          0.001 m
     * heading:          0.001 rad
     * l and r velocity: 0.1   m/s
     * l and r position: 0.005 m
     */
    null //VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
  );

    Field2d m_field = new Field2d();


  @Override 
  public void simulationPeriodic() {
    /* Pass the robot battery voltage to the simulated Talon FXs */
    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    /*
     * CTRE simulation is low-level, so SimCollection inputs
     * and outputs are not affected by SetInverted(). Only
     * the regular user-level API calls are affected.
     *
     * WPILib expects +V to be forward.
     * Positive motor output lead voltage is ccw. We observe
     * on our physical robot that this is reverse for the
     * right motor, so negate it.
     *
     * We are hard-coding the negation of the values instead of
     * using getInverted() so we can catch a possible bug in the
     * robot code where the wrong value is passed to setInverted().
     */
    m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                         -m_rightDriveSim.getMotorOutputLeadVoltage());

    /*
     * Advance the model by 20 ms. Note that if you are running this
     * subsystem in a separate thread or have changed the nominal
     * timestep of TimedRobot, this value needs to match it.
     */
    m_driveSim.update(0.02);

    /*
     * Update all of our sensors.
     *
     * Since WPILib's simulation class is assuming +V is forward,
     * but -V is forward for the right motor, we need to negate the
     * position reported by the simulation class. Basically, we
     * negated the input, so we need to negate the output.
     */
    m_leftDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                        m_driveSim.getLeftPositionMeters()
                    ));
    m_leftDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                        m_driveSim.getLeftVelocityMetersPerSecond()
                    ));
    m_rightDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                        -m_driveSim.getRightPositionMeters()
                    ));
    m_rightDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                        -m_driveSim.getRightVelocityMetersPerSecond()
                    ));
     
     m_gyroSim.setAngle(m_driveSim.getHeading().getDegrees());
  }

  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }
}
