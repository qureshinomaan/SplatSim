***
FAQ
***
* **Is ur_rtde realtime capable?**

Yes! you can define real-time priorities when instantiating the RTDEControl and RTDEReceive interfaces see the
:ref:`Real-time Setup Guide <realtime-setup-guide>`

* **Can I use ur_rtde interface through MATLAB?**

Yes! because ur_rtde have python bindings and MATLAB supports integration with Python,
these can be called directly through MATLAB. Please see the section :ref:`Use with MATLAB <use-with-matlab>`

* **How do I use my Robotiq gripper together with ur_rtde?**

Currently 3 different ways of controlling a Robotiq gripper with ur_rtde exists. Please see the section
:ref:`Use with Robotiq Gripper <use-with-robotiq-gripper>` for more information.

* **Can I pause the motion of the robot?**

If you use ur_rtde together with the ExternalControl UR cap or a custom script on the controller, you can pause
the motions using the teach pendant.

When uploading the default rtde_control script, you cannot pause the motion of the move, servo and speed commands.
However you can stop them. For the servo or speed movements you can stop the motion by using either servoStop()
for a servo command and speedStop() for a speed command.

For the move commands you need to set the async flag to true, in order to be able to stop them, using either
stopJ() or stopL(). see :ref:`Move Asynchronous Example <move-asynchronous-example>`.

* **Can I set a custom reference frame for the robot, eg. wrt. a feature?**

Currently it has not been implemented in the rtde_control script, but will be in an upcoming release. In the meantime please see
`Move with respect to a custom feature/frame <https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/urscript-move-with-respect-to-a-custom-featureframe-20115/>`_.