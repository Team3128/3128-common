# Vision

[`DynamicCamera`](DynamicCamera.java) reads AprilTags through PhotonVision and estimates where the
robot is, so the drivetrain can correct its odometry. Odometry from the wheels drifts as they slip.
Seeing a tag whose position on the field is known pulls the estimate back to the truth.

In simulation, the cameras see simulated tags, so vision code can be tested without a robot.
