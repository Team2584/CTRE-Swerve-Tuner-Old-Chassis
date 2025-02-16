package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/**
 * VisionSubsystem uses a limelight camera running MegaTag2 to update your pose estimation.
 * It reads values from NetworkTables, filters out bad pose observations, and if valid, passes
 * the vision measurement to the provided VisionConsumer.
 */
public class VisionSubsystem {

  // NetworkTables publishers/subscribers for the limelight/MegaTag2
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  // Consumer callback to pass along valid vision pose measurements.
  private final VisionConsumer consumer;

  // Latest target observation (for servoing, etc.)
  private TargetObservation latestTargetObservation;

  // Latest inputs from NetworkTables.
  private boolean connected;
  private PoseObservation[] poseObservations = new PoseObservation[0];
  private int[] tagIds = new int[0];

  // Alert if the camera is disconnected.
  private final Alert disconnectedAlert;

  /**
   * Constructs the VisionSubsystem.
   *
   * @param limelightName    The NetworkTable name for your limelight.
   * @param rotationSupplier A supplier for the current robot rotation (used to help MegaTag2).
   * @param consumer         A consumer that receives valid vision pose observations.
   */
  public VisionSubsystem(String limelightName, Supplier<Rotation2d> rotationSupplier, VisionConsumer consumer) {
    this.rotationSupplier = rotationSupplier;
    this.consumer = consumer;

    var table = NetworkTableInstance.getDefault().getTable(limelightName);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    disconnectedAlert = new Alert("Vision camera is disconnected.", AlertType.kWarning);
  }

  /**
   * Returns the target’s X angle (e.g. for simple servoing). If no observation exists, returns 0°.
   *
   * @return The horizontal angle to the target.
   */
  public Rotation2d getTargetX() {
    return latestTargetObservation != null ? latestTargetObservation.tx() : Rotation2d.fromDegrees(0.0);
  }

  /**
   * Call this method periodically (e.g. in Robot.periodic) to update vision inputs,
   * process pose observations, and pass valid measurements to the consumer.
   */
  public void periodic() {
    // Read and update the latest inputs from the camera.
    updateInputs();

    // Update our disconnected alert based on connection status.
    disconnectedAlert.set(!connected);

    // Loop over each pose observation and filter out invalid samples.
    for (PoseObservation observation : poseObservations) {
      // Reject if there are no tags, if a single tag is too ambiguous,
      // if the Z-coordinate error is too high, or if the pose is out-of-bounds.
      boolean rejectPose =
          observation.tagCount() == 0 ||
          (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.maxAmbiguity) ||
          Math.abs(observation.pose().getZ()) > VisionConstants.maxZError ||
          observation.pose().getX() < 0.0 ||
          observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength() ||
          observation.pose().getY() < 0.0 ||
          observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();

      if (rejectPose) {
        continue;
      }

      // Calculate standard deviations based on tag distance and count.
      double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
      double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor * VisionConstants.linearStdDevMegatag2Factor;
      double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor * VisionConstants.angularStdDevMegatag2Factor;
      
      // If you have camera-specific adjustments (for a single camera, use index 0)
      if (VisionConstants.cameraStdDevFactors.length > 0) {
        linearStdDev *= VisionConstants.cameraStdDevFactors[0];
        angularStdDev *= VisionConstants.cameraStdDevFactors[0];
      }

      // Pass the valid observation to the consumer.
      consumer.accept(
          observation.pose().toPose2d(),
          observation.timestamp(),
          VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
      );
    }
  }

  /**
   * Reads values from NetworkTables to update our camera inputs.
   */
  private void updateInputs() {
    // Check connection status: if the latency topic hasn't updated in the last 250ms, mark as disconnected.
    connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation using tx and ty.
    latestTargetObservation = new TargetObservation(
        Rotation2d.fromDegrees(txSubscriber.get()),
        Rotation2d.fromDegrees(tySubscriber.get())
    );

    // Publish our current robot orientation to assist MegaTag2.
    orientationPublisher.accept(new double[] {
        rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0
    });
    NetworkTableInstance.getDefault().flush();

    // Read pose observations from the MegaTag2 topic.
    Set<Integer> tagIdsSet = new HashSet<>();
    List<PoseObservation> poseObsList = new LinkedList<>();

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) {
        continue;
      }

      // Accumulate tag IDs (starting at index 11, stepping by 7)
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIdsSet.add((int) rawSample.value[i]);
      }

      // Compute vision timestamp in seconds.
      // rawSample.timestamp is in microseconds and rawSample.value[6] is the latency in milliseconds.
      double visionTimestamp = rawSample.timestamp * 1e-6 - rawSample.value[6] * 1e-3;

      // Parse the raw 3D pose.
      Pose3d rawPose3d = parsePose(rawSample.value);

      // Add this pose observation.
      poseObsList.add(new PoseObservation(
          visionTimestamp,            // Calculated vision timestamp (seconds)
          rawPose3d,                  // The raw 3D pose
          0.0,              // Ambiguity (set to zero since MegaTag2 resolves ambiguity)
          (int) rawSample.value[7],   // Tag count
          rawSample.value[9]          // Average tag distance
      ));
    }

    // Update our stored observations.
    poseObservations = poseObsList.toArray(new PoseObservation[0]);
    tagIds = new int[tagIdsSet.size()];
    int i = 0;
    for (int id : tagIdsSet) {
      tagIds[i++] = id;
    }
  }

  /**
   * LL 3D pose to botpose array.
   *
   * @param rawLLArray The raw array from the limelight.
   * @return A Pose3d built from the first six values of the array.
   */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])
        )
    );
  }

  /** Valid vision pose measurements. */
  @FunctionalInterface
  public static interface VisionConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** Record representing a simple target observation (for servoing, etc.). */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Record representing a full pose observation from vision. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance
  ) {}
}