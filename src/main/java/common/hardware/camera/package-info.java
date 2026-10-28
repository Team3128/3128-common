/**
 * Vision. {@link common.hardware.camera.DynamicCamera} reads AprilTags through PhotonVision and
 * works out where the robot is, so the drivetrain can correct its odometry. Odometry from the
 * wheels slowly drifts as they slip; seeing a tag whose position on the field is known pulls the
 * estimate back to the truth.
 *
 * <p>In simulation, the cameras see simulated tags, so vision code can be tested without a robot.
 */
package common.hardware.camera;
