package org.firstinspires.ftc.blackice.core.hardware.localization.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.blackice.core.hardware.localization.Localizer;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PinpointConfig implements LocalizerConfig {
   GoBildaPinpointDriver.EncoderDirection xPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
   GoBildaPinpointDriver.EncoderDirection yPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
   double xPodOffset = 0; // -36
   double yPodOffset = 0;
   DistanceUnit distanceUnit = DistanceUnit.INCH;
   GoBildaPinpointDriver. GoBildaOdometryPods podType =
       GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
   String name = "odo";
   
   public PinpointConfig name(String name) {
      this.name = name;
      return this;
   }
   
   /**
    * Can reverse the direction of each encoder.
    * @param xPodDirection FORWARD or REVERSED, X (forward) pod should increase when the robot is moving forward
    * @param yPodDirection FORWARD or REVERSED, Y (strafe) pod should increase when the robot is moving left
    */
   public PinpointConfig podDirection(
       GoBildaPinpointDriver.EncoderDirection xPodDirection,
       GoBildaPinpointDriver.EncoderDirection yPodDirection
   ) {
      this.xPodDirection = xPodDirection;
      this.yPodDirection = yPodDirection;
      return this;
   }
   
   /**
    * Sets the odometry pod positions relative to the point that the odometry computer tracks around.
    * <p>
    * The most common tracking position is the center of the robot.
    * <p>
    * The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number. the Y pod offset refers to how far forwards  (in mm) from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.
    *
    * @param xPodOffset how sideways from the center of the robot is the X (forward) pod?
    * Left increases
    * @param yPodOffset how far forward from the center of the robot is the Y (Strafe) pod? forward increases
    */
   public PinpointConfig podOffset(double xPodOffset, double yPodOffset) {
      this.xPodOffset = xPodOffset;
      this.yPodOffset = yPodOffset;
      return this;
   }
   
   public PinpointConfig distanceUnit(
       DistanceUnit distanceUnit) {
      this.distanceUnit = distanceUnit;
      return this;
   }
   
   /**
    * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.<br><br>
    * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
    */
   public PinpointConfig pods(GoBildaPinpointDriver.GoBildaOdometryPods pods) {
      this.podType = pods;
      return this;
   }
   
   @Override
   public Localizer build(HardwareMap hardwareMap) throws InterruptedException {
      return new PinpointLocalizer(hardwareMap, this);
   }
}
