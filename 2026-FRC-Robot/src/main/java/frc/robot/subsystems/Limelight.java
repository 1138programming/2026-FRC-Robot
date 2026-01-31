package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

import static frc.robot.Constants.LimelightConstants.TagConstants.OffsetConstants.*;
import static frc.robot.Constants.FieldConstants.TagIDConstants.*;

//import static frc.robot.Constants.LimelightConstants.*;
//conostants for later
public class Limelight extends SubsystemBase {
    
    
    private NetworkTable LimelightOneTable;
    private double[] botpose;
    private Pose2d poseEstimate = new Pose2d();
    private final Field2d field = new Field2d();

    private final String limelightName;

    //Robot position in fieldspace from AprilTag pipeline
    private double botPoseX; //bot translation in X -> set to wpiblue for now
    private double botPoseY; //bot translation in Y -> set to wpiblue for now
    private double botPoseZ; //bot translation in Z -> set to wpiblue for now

    //Robot space camera position
    private double roll; //bot rotation around X
    private double pitch; //bot rotation around Y
    private double yaw; //bot rotation around Z

    //Limelight Camera view data
    //resolves to target tix (most prevalent tag)
    private double tx; //horizontal offset
    private double ty; //vertical offset

    //View of tag data
    private double numberOfTargetsSeen; //number of targets seen
    private double tagSpan; //span between detected tags
    private double averageDistance; //average distance to detected tags
    private double Area; //average area of detected tags

    private double ID; //tag id in primary view
    private int priorityID; //highest priority tag id in view - ignore all other april tags

    //other 
    private double latency;
    private int isTargetsDetected;

    private final DoubleArrayPublisher orientationPublisher;
    private final DoubleArrayPublisher filterPublisher;
  

   

  public Limelight(String limelightName) {
    
    this.limelightName = limelightName; //should be instatialized with a constant value passed 
    LimelightOneTable = NetworkTableInstance.getDefault().getTable("limelight");

    botpose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);

    if (botpose.length != 0) {
      botPoseX = botpose[0];
      botPoseY = botpose[1];
      botPoseZ = botpose[2];
      roll = botpose[3];
      pitch = botpose[4];
      yaw = botpose[5];
      latency = botpose[6];
      numberOfTargetsSeen = botpose[7];
      tagSpan = botpose[8];
      averageDistance = botpose[9];
      Area = botpose[10];
      ID = botpose[11];

    }

    priorityID = -1;
    //LimelightOneTable.getEntry("priorityid").setNumber(priorityID);
    orientationPublisher = LimelightOneTable.getDoubleArrayTopic("robot_orientation_set").publish();
    filterPublisher =    LimelightOneTable.getDoubleArrayTopic("fiducial_id_filters_set").publish();


  //stores the data for the LL
  }

  public void periodic() {
    //Gets whatever the vaaibale names are
    isTargetsDetected = (int)LimelightOneTable.getEntry("tv").getInteger(0);
    tx = LimelightOneTable.getEntry("tx").getDouble(0);  
    ty = LimelightOneTable.getEntry("ty").getDouble(0);  
    Area = LimelightOneTable.getEntry("ta").getDouble(0); 
    ID = LimelightOneTable.getEntry("tid").getDouble(0);  
    

   Area = LimelightOneTable.getEntry("TA").getDouble(0); 
   ID = LimelightOneTable.getEntry("TID").getDouble(0);  
   
   LimelightOneTable.getEntry("TID").getDoubleArray(new double[11]);


   filterPublisher.accept(new double[]{3,4});



   //botPose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new double[61]);

    if (botpose.length != 0) {
      botPoseX = botpose[0];
      botPoseY = botpose[1];
      botPoseZ = botpose[2];
      roll = botpose[3];
      pitch = botpose[4];
      yaw = botpose[5];
      latency = botpose[6];
      numberOfTargetsSeen = botpose[7];
      tagSpan = botpose[8];
      averageDistance = botpose[9];
      Area = botpose[10];
      ID = botpose[11];

    }
    
  }

  //botpose getters

  public double getBotPoseX() {
    return botPoseX;
  }

  public double getBotPoseY() {
    return botPoseY;
  }

  public double getBotPoseZ() {
    return botPoseZ;
  }

  public double getRoll() {
    return roll;
  }

  public double getPitch() {
    return pitch;
  }

  public double getYaw() {
    return yaw;
  }

  public double getLatency() {
    return latency;
  }

  public double getNumberOfTargetsSeen() {
    return numberOfTargetsSeen;
  }

  public double getTagSpan() {
    return tagSpan;
  }

  public double getAverageDistance() {
    return averageDistance;
  }

  public double getArea() {
    return Area;
  }

  public double getID() {
    return ID;
  }

  public void setPriorityID(int id) {
    priorityID = id;
    LimelightOneTable.getEntry("priorityid").setNumber(priorityID);
  }

  //figure out if this works or not. Is priority id tracked after all tags are processed. 
  public void resetPriorityID() {
    priorityID = -1;
    LimelightOneTable.getEntry("priorityid").setNumber(priorityID);
  }

  public int getPriorityID() {
    return priorityID;
  }

  //everything else

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public void setFiducial3DOffset(double x, double y, double z) {
    LimelightHelpers.setFiducial3DOffset(limelightName, x, y, z);
  }

  public void resetFiducial3DOffset() {
    LimelightHelpers.setFiducial3DOffset(limelightName, 0, 0, 0);
  }

  public double getOffsetTx() {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getOffsetTy() {
    return LimelightHelpers.getTY(limelightName);
  }

  //specific offset getters for FRC 2026 april tags - these are specific values to the game 

  /**
   * Returns the Tx offset from the central hub tag offset to calculate angle to the middle point of the hub's entrance. Used for aiming and projectile motion calculations for shooting fuel into hub.
   * Assumes that the targeted point of interest is an april tag centered on the hub (ex. ID 10 for red, ID 26 for blue)
   * @apiNote Forward offset: -0.5969 meters,  Height offset: 0.70485 meters
   * @return Tx with offset (degrees)
   */
  public double getHubCenterTagtoOffsetHubCenterTx() {
    setFiducial3DOffset(kX_HubCenterTagtoHubCenterMeters, 0, kY_HubCenterTagtoHubScoreHeightMeters);
    double result = getOffsetTx();
    resetFiducial3DOffset();
    return result;
  }

   /**
   * Returns the Tx offset from the central hub tag offset to calculate angle to the middle point of the hub's entrance. Used for aiming and projectile motion calculations for shooting fuel into hub.
   * Assumes that the targeted point of interest is an april tag centered on the hub (ex. ID 10 for red, ID 26 for blue)
   * @apiNote Forward offset: -0.5969 meters,  Height offset: 0.70485 meters
   * @return Ty with offset (degrees)
   */
  public double getHubCenterTagtoOffsetHubCenterTy() {
    setFiducial3DOffset(kX_HubCenterTagtoHubCenterMeters, 0, kY_HubCenterTagtoHubScoreHeightMeters);
    double result = getOffsetTy();
    resetFiducial3DOffset();
    return result;
  }

    /**
   * Calculates the distance from the limelight to the middle point of the hub's entrance and returns it.
   * Used to calculate any other distance values as a hypotoneuse
   * 
   * @return Distance to middle point hub entrance (meters), -1.0 if Hub center tag not found
   * 
   */
  public double getHubCenterTagtoOffsetHubCenterDistancetoCamera() {
    setFiducial3DOffset(kX_HubCenterTagtoHubCenterMeters, 0, kY_HubCenterTagtoHubScoreHeightMeters);
    LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    for (LimelightHelpers.RawFiducial fiducial : fiducials) {
      int id = fiducial.id;                    // Tag ID
      double distToCamera = fiducial.distToCamera;  // Distance to camera

      //NOTE: check if we require any other IDs
      if (id == kHubCenterTagRed || id == kHubCenterTagBlue) {
        resetFiducial3DOffset();
        return distToCamera;
      }
    }
    resetFiducial3DOffset();
    return -1.0; //default value, no specified ID found. 
  }

  public double getHubCenterTagtoOffsetHubCenterDistancetoRobot() {
    setFiducial3DOffset(kX_HubCenterTagtoHubCenterMeters, 0, kY_HubCenterTagtoHubScoreHeightMeters);
    LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
    for (LimelightHelpers.RawFiducial fiducial : fiducials) {
      int id = fiducial.id;                    // Tag ID
      double distToRobot = fiducial.distToRobot;  // Distance to camera

      //NOTE: check if we require any other IDs
      if (id == kHubCenterTagRed || id == kHubCenterTagBlue) {
        resetFiducial3DOffset();
        return distToRobot;
      }
    }
    resetFiducial3DOffset();
    return -1.0; //default value, no specified ID found. 
  }

  public int getIsTargetsDetected() {
    return isTargetsDetected;
  }
  
  //Limelight Pose 3D compilation 
  /**
   * Compiles the Limelight botpose_wpiblue array into a Pose3d object
   * 
   * @return Pose3d of roll pitch, yaw Rotation3d and x, y, z translation
   */ 
  public Pose3d LimelightToPose3d() {
    Rotation3d rot3d = new Rotation3d(getRoll(), getPitch(), getYaw());
    return new Pose3d(getBotPoseX(), getBotPoseY(), getBotPoseZ(), rot3d);
  }


//botpose
public double[] getBotPose() {
    return botpose;
  }
}




