package frc.robot.Auto;

import com.pathplanner.lib.auto.AutoBuilder.QuadFunction;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;


public class AutoBuilder {
    private static boolean configured = false;

  private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
  private static Supplier<Pose2d> getPose;
  private static Consumer<Pose2d> resetPose;
  private static BooleanSupplier shouldFlipPath;

  // Pathfinding builders
  private static boolean pathfindingConfigured = false;
  private static QuadFunction<Pose2d, PathConstraints, Double, Double, Command>
      pathfindToPoseCommandBuilder;
  private static TriFunction<PathPlannerPath, PathConstraints, Double, Command>
      pathfindThenFollowPathCommandBuilder;
/**
   * Configures the AutoBuilder for a differential drivetrain using a RAMSETE path follower.
   *
   * @param poseSupplier a supplier for the robot's current pose
   * @param resetPose a consumer for resetting the robot's pose
   * @param speedsSupplier a supplier for the robot's current chassis speeds
   * @param output a consumer for setting the robot's chassis speeds
   * @param b Tuning parameter (b &gt; 0 rad^2/m^2) for which larger values make convergence more
   *     aggressive like a proportional term.
   * @param zeta Tuning parameter (0 rad^-1 &lt; zeta &lt; 1 rad^-1) for which larger values provide
   *     more damping in response.
   * @param replanningConfig Path replanning configuration
   * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
   *     the field. This will maintain a global blue alliance origin.
   * @param driveSubsystem the subsystem for the robot's drive
   * @throws AutoBuilderException if AutoBuilder has already been configured
   */
  public static void configureRamsete(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Supplier<ChassisSpeeds> speedsSupplier,
      Consumer<ChassisSpeeds> output,
      double b,
      double zeta,
      ReplanningConfig replanningConfig,
      BooleanSupplier shouldFlipPath,
      Subsystem driveSubsystem) {
    if (configured) {
      throw new AutoBuilderException(
          "Auto builder has already been configured. Please only configure auto builder once");
    }

    AutoBuilder.pathFollowingCommandBuilder =
        (path) ->
            new FollowPathRamsete(
                path,
                poseSupplier,
                speedsSupplier,
                output,
                b,
                zeta,
                replanningConfig,
                shouldFlipPath,
                driveSubsystem);
    AutoBuilder.getPose = poseSupplier;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.configured = true;
    AutoBuilder.shouldFlipPath = shouldFlipPath;

    AutoBuilder.pathfindToPoseCommandBuilder =
        (pose, constraints, goalEndVel, rotationDelayDistance) ->
            new PathfindRamsete(
                pose.getTranslation(),
                constraints,
                goalEndVel,
                poseSupplier,
                speedsSupplier,
                output,
                b,
                zeta,
                replanningConfig,
                driveSubsystem);
    AutoBuilder.pathfindThenFollowPathCommandBuilder =
        (path, constraints, rotationDelayDistance) ->
            new PathfindThenFollowPathRamsete(
                path,
                constraints,
                poseSupplier,
                speedsSupplier,
                output,
                b,
                zeta,
                replanningConfig,
                shouldFlipPath,
                driveSubsystem);
    AutoBuilder.pathfindingConfigured = true;
  }

}
