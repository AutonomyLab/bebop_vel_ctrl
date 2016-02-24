^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bebop_vel_ctrl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2016-02-23)
------------------
* Add trajectory plan executer
* Add zero_vxy param for hovering when setpoint's vx and vy are zero
* Add param for safety mode, change yaw control mode
* Contributors: Mani Monajjemi

1.1.0 (2016-02-12)
------------------
* Change default value of abs_yaw to false, communicate that to the user
* Less verbose output
* Set max velocity for xy to 2.0 m/s
* Contributors: Mani Monajjemi

1.0.0 (2016-02-10)
------------------
* Initial stable version of velocity controller for Parrot Bebop 1
* Vel control for x/y DOFs
* Vel control and abs control for z/yaw DOFs
* joystick teleop
* Dynamic reconfigure for PIDs
* Contributors: Mani Monajjemi, Sepehr Mohaimanianpour
