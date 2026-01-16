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

    private double botPoseX;
    private double botPoseY;
    private double botPoseZ;
    private double roll;
    private double pitch;
    private double yaw;
    private double latency;
    private double numberOfTargetsSeen;
    private double tagSpan;
    private double averageDistance;
    private double Area;
    private double tableOne;
    private double target;
    private double ID;


   

  public Limelight() {
    
    LimelightOneTable = NetworkTableInstance.getDefault().getTable("limelight");

    botpose = LimelightOneTable.getEntry("botpose_wpiblue").getDoubleArray(new double[11]);

    tableOne = LimelightOneTable.getEntry("tableOne").getDouble(0);

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
  //stores the data for the LL
  
  }
  public void periodic() {
    //Gets whatever the vaaibale names are
    target = LimelightOneTable.getEntry("tv").getDouble(0);
   botPoseX = LimelightOneTable.getEntry("TX").getDouble(0);  
   botPoseY = LimelightOneTable.getEntry("TY").getDouble(0);  
   Area = LimelightOneTable.getEntry("TA").getDouble(0); 
   ID = LimelightOneTable.getEntry("TID").getDouble(0);  
   LimelightOneTable.getEntry("TID").getDoubleArray(new double[41]);


}

}
  



