# Driver controllers

- [`NAR_XboxController`](NAR_XboxController.java)
- [`NAR_Joystick`](NAR_Joystick.java)
- [`NAR_ButtonBoard`](NAR_ButtonBoard.java), the team's button board

Each class turns its buttons into `Trigger`s to bind commands to, applies a small deadband to its
sticks so they read 0 when released, and makes up and forward positive.

The number passed to the constructor is the controller's port in the Driver Station's USB tab. In
simulation, drag a controller onto that port in the sim window's Joysticks panel.
