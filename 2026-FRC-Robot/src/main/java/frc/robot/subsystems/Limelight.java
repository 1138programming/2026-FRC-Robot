package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

//import static frc.robot.Constants.LimelightConstants.*;
//conostants for later
public class Limelight extends SubsystemBase {
    
    
    private NetworkTable LimelightOneTable;
    private double[] botpose;
    private Pose2d poseEstimate = new Pose2d();
    private final Field2d field = new Field2d();

    //Robot position in fieldspace from AprilTag pipeline
    private double botPoseX; //bot translation in X
    private double botPoseY; //bot translation in Y
    private double botPoseZ; //bot translation in Z

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
  

   

  public Limelight() {
    
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

  public int getIsTargetsDetected() {
    return isTargetsDetected;
  }


//botpose
public double[] getBotPose() {
    return botpose;
  }
}




