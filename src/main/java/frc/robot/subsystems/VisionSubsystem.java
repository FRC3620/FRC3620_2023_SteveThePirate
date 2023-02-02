package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  public VisionSubsystem() {
    super();
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException ex) {
      System.out.println("unable to load file");
    }
    Thread visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
  }

  public Config atagCamConfig = new Config(0.1524,573.18, 573.08, 334.47, 180.12);
  public double atag1TransformX;
  public double atag1TransformY;
  public double atag1TransformZ;
  public boolean getTargetTransform = true;

  public static Double targetOneX = null;
  public static AllAprilTagsInPicture allAprilTagsInPicture = null;

  static AprilTagFieldLayout fieldLayout;


  @Override
  public void periodic() {
    if (targetOneX == null) {
      SmartDashboard.putString("TagX", "null");
    } else if (targetOneX > 0.5) {
      SmartDashboard.putString("TagX", "right");
    } else {
      SmartDashboard.putString("TagX", "left");
    }
    SmartDashboard.putString("TagXValue", "" + targetOneX);
  }

  public Transform3d tag1Transform;

  public void apriltagVisionThreadProc() {
    AprilTagDetector detector = new AprilTagDetector();
    detector.addFamily("tag16h5", 0);
    AprilTagPoseEstimator atagPoseEstimator = new AprilTagPoseEstimator(atagCamConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(640, 360);
    camera.setBrightness(50);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("detect", 640, 360);

    // Mats are very memory expensive. Lets reuse this Mat.
    Mat mat = new Mat();
    Mat grayMat = new Mat();
    ArrayList<Integer> tags = new ArrayList<>();

    //
    Scalar outlineColor = new Scalar(0, 255, 0);
    Scalar xColor = new Scalar(0, 0, 255);

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }
      double width = mat.width();
      SmartDashboard.putNumber("width?", width);

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);
      tags.clear();

      AllAprilTagsInPicture allAprilTagsInPictureUnderConstruction = new AllAprilTagsInPicture();
      Double temp = null;
      for (AprilTagDetection detection : detections) {
        tags.add(detection.getId());

        Transform3d transform3d = atagPoseEstimator.estimate(detection);

        allAprilTagsInPictureUnderConstruction.add(detection, transform3d);

        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), xColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), xColor, 2);
        Imgproc.putText(mat, Integer.toString(detection.getId()), new Point (cx + ll, cy), Imgproc.FONT_HERSHEY_SIMPLEX, 1, xColor, 3);
      }

      
      AprilTagDetection tag1Detection = allAprilTagsInPictureUnderConstruction.getDetection(1);
      if (tag1Detection != null) {
        targetOneX = tag1Detection.getCenterX()/(width-1);
        tag1Transform = allAprilTagsInPictureUnderConstruction.getTransform3d(1);

        SmartDashboard.putNumber("LAT.tag1posex", tag1Transform.getX()*39.3701);
        SmartDashboard.putNumber("LAT.tag1posey", tag1Transform.getY()*39.3701);
        SmartDashboard.putNumber("LAT.tag1posez", tag1Transform.getZ()*39.3701);
        SmartDashboard.putNumber("LAT.angle", Math.toDegrees (Math.atan((tag1Transform.getX()*39.3701)/(tag1Transform.getZ()*39.3701))));
        SmartDashboard.putNumber("LAT.distance", Math.sqrt(Math.pow(tag1Transform.getZ()*39.3701-36, 2) + Math.pow(tag1Transform.getX()*39.3701, 2)));
      }

      SmartDashboard.putString("tag", tags.toString());

      allAprilTagsInPicture = allAprilTagsInPictureUnderConstruction;

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    detector.close();
  }

  public Transform3d getTag1Transform() {
    return tag1Transform;
  }

  public void clearTag1Transform() {
    tag1Transform = null;
  }

  public static class AllAprilTagsInPicture{
    Map<Integer, AprilTagDetection> detections;
    Map<Integer, Transform3d> transforms;

    AllAprilTagsInPicture() {
      detections = new HashMap<>();
      transforms = new HashMap<>();
    }

    void add (AprilTagDetection detection, Transform3d transform) {
      Integer id = detection.getId();
      detections.put(id, detection);
      transforms.put(id, transform);
    }

    public int size() {
      return detections.size();
    }

    public AprilTagDetection getDetection(int i) {
      return detections.get(i);
    }

    public Transform3d getTransform3d(int i){
      return transforms.get(i);
    }

    public Integer getIdOfClosestTag() {
      int size = size();
      if (size == 0) return null;
      var keys = new ArrayList<>(transforms.keySet());
      var key0 = keys.get(0);
      // TODO NEEDS WORK!
      return key0;
    }

  }

  public static Translation3d getTranslation3dForTag(int tag) {
    var footemp = fieldLayout.getTagPose(tag);
    if (footemp.isPresent()) 
    {
      Translation3d transform3d = footemp.get().getTranslation();
      SmartDashboard.putNumber("whereami.TagPosex", transform3d.getX());
      SmartDashboard.putNumber("whereami.TagPosey", transform3d.getY());
      SmartDashboard.putNumber("whereami.TagPosez", transform3d.getZ());
      return transform3d;
    }
    return null;
  }
}