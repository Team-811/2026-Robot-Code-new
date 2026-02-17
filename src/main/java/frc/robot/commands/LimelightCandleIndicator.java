// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.LimelightHelpers;
// import frc.robot.subsystems.CandleLED;
// import frc.robot.subsystems.LimelightShooter;

// /**
//  * Default/continuous indicator: lights the CANdle green when the Limelight shooter sees a
//  * target on the desired pipeline, otherwise turns the strip off.
//  * 
//  * Why a Command?
//  * - Keeps logic in the scheduler so it runs periodically without blocking other code.
//  * - Owns the CandleLED requirement so no other command can fight over LED colors.
//  */
// public class LimelightCandleIndicator extends Command {
//   private final CandleLED candle;
//   private final int watchedPipeline;

//   /**
//    * @param limelight The shooter Limelight subsystem (data source).
//    * @param candle The CANdle LED subsystem (output).
//    * @param watchedPipeline Pipeline index we care about; only light up when this is active.
//    */
//   public LimelightCandleIndicator(LimelightShooter limelight, CandleLED candle, int watchedPipeline) {
//     this.candle = candle;
//     this.watchedPipeline = watchedPipeline;
//     addRequirements(candle); // Prevent competing LED commands.
//   }

//   @Override
//   public void execute() {
//     // Treat tv > 0.5 as "valid target" to avoid equality issues with doubles.
//     boolean hasTarget = LimelightShooter.hasTarget();
//     int currentPipeline =
//         (int) Math.round(LimelightHelpers.getCurrentPipelineIndex(LimelightShooter.LL_NAME));
//     boolean onWatchedPipeline = currentPipeline == watchedPipeline;

//     // Light green only when the right pipeline is active AND a target is visible.
//     if (onWatchedPipeline && hasTarget) {
//       candle.setGreen();
//     } else {
//       candle.setOff();
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     // Always leave LEDs off when this command stops so the strip does not get stuck on.
//     candle.setOff();
//   }

//   @Override
//   public boolean isFinished() {
//     // Run continuously as a status indicator.
//     return false;
//   }
// }
