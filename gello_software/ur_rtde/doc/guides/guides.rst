******
Guides
******
This section contains guides for how to use the ur_rtde interface.

.. _realtime-setup-guide:

.. role:: bash(code)
   :language: bash


Real-time Setup Guide
=====================
ur_rtde makes it possible to specify a desired real-time priority for the RTDEControlInterface and RTDEReceiveInterface.
First you must make sure your OS supports a real-time kernel and install it, if not already available. In the following
sections you can find how to set it up on various commonly used operating systems.

.. tip::

    It is highly recommended that you use ur_rtde in combination with a real-time kernel, this makes sure that if the system
    experience a high load, ur_rtde would still be able to control the robot in a precise and safe manner.

Linux
-----
The linux kernel is not real-time capable by default, and therefore one has to be installed.

Ubuntu 22.04
~~~~~~~~~~~~
For Ubuntu 22.04 a real-time beta kernel is already installed, but it has to be enabled. See more
information here: `Real-time Ubuntu 22.04 LTS Beta <https://ubuntu.com/blog/real-time-ubuntu-released>`_

How to enable the real-time kernel beta
"""""""""""""""""""""""""""""""""""""""
The beta kernel is available for free for personal use via Ubuntu Advantage for Infrastructure (UA-I)
To attach your personal machine to a UA subscription, please run:

.. code-block:: shell

    ua attach <free token>

where the <free token> is found on your Ubuntu one account associated with the free UA subscription.

Make sure you have at least version 27.8 of ubuntu-advantage-tools. You can check the version with:

.. code-block:: shell

    ua version

To upgrade ubuntu-advantage-tools to 27.8 in Jammy Jellyfish, please run:

.. code-block:: shell

    sudo apt install ubuntu-advantage-tools=27.8~22.04.1

To enable the real-time beta kernel, run:

.. code-block:: shell

    ua enable realtime-kernel --beta

Now reboot the system and the realtime kernel should be loaded.

.. warning::
    Please note you will need to manually configure grub to revert back to your original kernel after enabling real-time.

    For more information: ua help realtime-kernel

Ubuntu 20.04 / 18.04 / 16.04
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To get real-time support into an ubuntu system, the following steps have to be performed:

1. Get the sources of a real-time kernel
2. Compile the real-time kernel
3. Install real-time kernel

.. note::
   NVIDIA Drivers are not supported on PREEMPT_RT kernels!

Install dependencies
""""""""""""""""""""
To build the kernel, you will need a couple of tools available on your system. You can install them using

.. code-block:: shell

   sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev zstd libncurses-dev dwarves gawk flex bison

To continue with this tutorial, please create a temporary folder and navigate into it.
You should have sufficient space (around 25GB) there, as the extracted kernel sources take much space.
After the new kernel is installed, you can delete this folder again.

In this example we will use a temporary folder inside our home folder:

.. code-block:: shell

   mkdir -p ${HOME}/rt_kernel_build
   cd ${HOME}/rt_kernel_build

Getting the sources for a real-time kernel
""""""""""""""""""""""""""""""""""""""""""
Then, you have to decide which kernel version to use. To find the one you are using currently, use :bash:`uname -r`.
Real-time patches are only available for select kernel versions,
see https://www.kernel.org/pub/linux/kernel/projects/rt/. We recommend choosing the version closest to the one you
currently use. If you choose a different version, simply substitute the numbers. Having decided on a version, use curl
to download the source files:


.. admonition:: Ubuntu 16.04
  :class: note

    Ubuntu 16.04 was tested with the kernel version 4.14.12:

    .. code-block:: shell

        curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.xz
        curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.sign
        curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz
        curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.sign

.. admonition:: Ubuntu 18.04
  :class: note

    Ubuntu 18.04 was tested with the kernel version 5.4.19:

    .. code-block:: shell

        curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.19.tar.xz
        curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.19.tar.sign
        curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.19-rt10.patch.xz
        curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.19-rt10.patch.sign

.. admonition:: Ubuntu 20.04
  :class: note

    Ubuntu 20.04 was tested with the kernel version 5.9.1:

    .. code-block:: shell

        curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
        curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
        curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
        curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign

To unzip the downloaded files do

.. code-block:: shell

   xz -d *.xz

Verification
""""""""""""
.. note::
    This step is optional but recommended!

The .sign files can be used to verify that the downloaded files were not corrupted or tampered with. The steps shown
here are adapted from the `Linux Kernel Archive <https://www.kernel.org/signature.html>`_, see the linked page for more
details about the process.

You can use gpg2 to verify the .tar archives:

.. code-block:: shell

    gpg2 --verify linux-*.tar.sign
    gpg2 --verify patch-*.patch.sign

If your output is similar to the following:

.. code-block:: none

    $ gpg2 --verify linux-*.tar.sign
    gpg: assuming signed data in 'linux-4.14.12.tar'
    gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
    gpg: Can't check signature: No public key

You have to first download the public key of the person who signed the above file. As you can see from the above output,
it has the ID 6092693E. You can obtain it from the key server:

.. code-block:: shell

    gpg2  --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6092693E

Similarly for the patch:

.. code-block:: shell

    gpg2 --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 2872E4CC

Note that keys for other kernel version might have different IDs, you will have to adapt accordingly.

Having downloaded the keys, you can now verify the sources. Here is an example of a correct output:

.. code-block:: shell

    $ gpg2 --verify linux-*.tar.sign
    gpg: assuming signed data in 'linux-4.14.12.tar'
    gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
    gpg: Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" [unknown]
    gpg:                 aka "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
    gpg:                 aka "Greg Kroah-Hartman (Linux kernel stable release signing key) <greg@kroah.com>" [unknown]
    gpg: WARNING: This key is not certified with a trusted signature!
    gpg:          There is no indication that the signature belongs to the owner.
    Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E

See `Linux Kernel Archive <https://www.kernel.org/signature.html>`_ for more information about the warning.

Compilation
"""""""""""
Once you are sure the files were downloaded properly, you can extract the source code and apply the patch:

.. code-block:: shell

    tar xf linux-*.tar
    cd linux-*/
    patch -p1 < ../patch-*.patch

Next copy your currently booted kernel configuration as the default config for the new real time kernel:

.. code-block:: shell

    cp -v /boot/config-$(uname -r) .config

Now you can use this config as the default to configure the build:

.. code-block:: shell

    make olddefconfig
    make menuconfig

The second command brings up a terminal interface in which you can configure the preemption model.
For the preemption model select Fully Preemptible Kernel:

.. code-block:: shell

   Preemption Model
      1. No Forced Preemption (Server) (PREEMPT_NONE)
    > 2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
      3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
      4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
      5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)
    choice[1-5]: 5

After that navigate to Cryptographic API > Certificates for signature checking (at the very bottom of the list) >
Provide system-wide ring of trusted keys > Additional X.509 keys for default system keyring

Remove the “debian/canonical-certs.pem” from the prompt and press Ok. Save this configuration to :bash:`.config` and
exit the TUI.

Now you are ready to compile the kernel. As this is a lengthy process, set the multithreading option -j to the number
of your CPU cores:

.. code-block:: shell

    make -j$(nproc) deb-pkg


Installation
""""""""""""
After building, install the linux-headers and linux-image packages in the parent folder
(only the ones without the -dbg in the name).

.. code-block:: shell

    sudo dpkg -i ../linux-headers-*.deb ../linux-image-*.deb

Verifying the new kernel
""""""""""""""""""""""""
Restart your system. The Grub boot menu should now allow you to choose your newly installed kernel. To see which one is
currently being used, see the output of the :bash:`uname -a` command. It should contain the string PREEMPT RT and the version
number you chose. Additionally, :file:`/sys/kernel/realtime` should exist and contain the the number :bash:`1`.

Setup user privileges to use real-time scheduling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To be able to set real-time priority on threads with user privileges you'll have to change the user's limits by
changing :file:`/etc/security/limits.conf` (See the manpage for details).

It is recommended to setup a group for real-time users instead of writing a fixed username into the config file:

.. code-block:: shell

    sudo groupadd realtime
    sudo usermod -aG realtime $(whoami)

Afterwards, add the following limits to the *realtime* group in :file:`/etc/security/limits.conf` contains:

.. code-block:: shell

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

You need to log out and in again or simply reboot in order for the new limits to take effect.

This part of the guide was inspired by the guide from the universal robots urcl client library git repository found
`here <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/real_time.md>`_. as well as
the guide from the Franka Control Interface documentation found here:
`Setting up the real-time kernel <https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_.

Windows
-------
The Windows NT kernel is real-time capable by default, this means we do not need to setup anything special to
execute ur_rtde with real-time priority, although you might have to run your program as Administrator.
On Windows the real-time priorities are set differently than on Linux and in another range.

Learn more about the real-time priorities on windows here: `Scheduling Priorities <https://docs.microsoft.com/en-us/windows/win32/procthread/scheduling-priorities>`_

Setting a real-time priority
----------------------------
The real-time priority of the RTDEControl and RTDEReceiveInterface can be set with an integer specified
in the constructor of the interfaces. Like so:

.. code-block:: c++

    // ur_rtde real-time priorities
    int rt_receive_priority = 90;
    int rt_control_priority = 85;

    RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
    RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);

    // Set application real-time priority
    RTDEUtility::setRealtimePriority(80);

On linux the priority range is (0-99) where 99 is the highest priority available. However be aware that a priority
of 99 might make the OS unstable. If no priority is specified the interface will default to the safe maximum priority
of 90. If a negative priority is specified, real-time priority will be disabled.

On Windows the REALTIME_PRIORITY_CLASS is set for the process, this means that the priority range is (16-31), where
the priority levels are:

+-------------------------------+---------------+
| Thread priority level         | Base priority |
+===============================+===============+
| THREAD_PRIORITY_IDLE          | 16            |
+-------------------------------+---------------+
| THREAD_PRIORITY_LOWEST        | 22            |
+-------------------------------+---------------+
| THREAD_PRIORITY_BELOW_NORMAL  | 23            |
+-------------------------------+---------------+
| THREAD_PRIORITY_NORMAL        | 24            |
+-------------------------------+---------------+
| THREAD_PRIORITY_ABOVE_NORMAL  | 25            |
+-------------------------------+---------------+
| THREAD_PRIORITY_HIGHEST       | 26            |
+-------------------------------+---------------+
| THREAD_PRIORITY_TIME_CRITICAL | 31            |
+-------------------------------+---------------+

Also see the more complete C++ real-time control example under :file:`examples/cpp/realtime_control_example.cpp`

.. code-block:: c++

    #include <ur_rtde/rtde_control_interface.h>
    #include <ur_rtde/rtde_receive_interface.h>
    #include <ur_rtde/rtde_io_interface.h>
    #include <thread>
    #include <chrono>

    using namespace ur_rtde;
    using namespace std::chrono;

    // interrupt flag
    bool running = true;
    void raiseFlag(int param)
    {
      running = false;
    }

    std::vector<double> getCircleTarget(const std::vector<double> &pose, double timestep, double radius=0.075, double freq=1.0)
    {
      std::vector<double> circ_target = pose;
      circ_target[0] = pose[0] + radius * cos((2 * M_PI * freq * timestep));
      circ_target[1] = pose[1] + radius * sin((2 * M_PI * freq * timestep));
      return circ_target;
    }

    int main(int argc, char* argv[])
    {
      // Setup parameters
      std::string robot_ip = "localhost";
      double rtde_frequency = 500.0; // Hz
      double dt = 1.0 / rtde_frequency; // 2ms
      uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
      int ur_cap_port = 50002;

      // ur_rtde realtime priorities
      int rt_receive_priority = 90;
      int rt_control_priority = 85;

      RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
      RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);

      // Set application realtime priority
      RTDEUtility::setRealtimePriority(80);

      // Move parameters
      double vel = 0.5;
      double acc = 0.5;

      // Servo control parameters
      double lookahead_time = 0.1;
      double gain = 600;

      signal(SIGINT, raiseFlag);

      double time_counter = 0.0;

      // Move to init position using moveL
      std::vector<double> actual_tcp_pose = rtde_receive.getActualTCPPose();
      std::vector<double> init_pose = getCircleTarget(actual_tcp_pose, time_counter);
      rtde_control.moveL(init_pose, vel, acc);

      try
      {
        while (running)
        {
          steady_clock::time_point t_start = rtde_control.initPeriod();
          std::vector<double> servo_target = getCircleTarget(actual_tcp_pose, time_counter);
          rtde_control.servoL(servo_target, vel, acc, dt, lookahead_time, gain);
          rtde_control.waitPeriod(t_start);
          time_counter += dt;
        }
        std::cout << "Control interrupted!" << std::endl;
        rtde_control.servoStop();
        rtde_control.stopScript();
      }
      catch(std::exception& e)
      {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
      }
      catch(...)
      {
        std::cerr << "Exception of unknown type!\n";
      }
      return 0;
    }

And in Python:

.. code-block:: python

   from rtde_control import RTDEControlInterface as RTDEControl
   from rtde_receive import RTDEReceiveInterface as RTDEReceive
   import datetime
   import math
   import os
   import psutil
   import sys

   def getCircleTarget(pose, timestep, radius=0.075, freq=1.0):
       circ_target = pose[:]
       circ_target[0] = pose[0] + radius * math.cos((2 * math.pi * freq * timestep))
       circ_target[1] = pose[1] + radius * math.sin((2 * math.pi * freq * timestep))
       return circ_target

   # Parameters
   vel = 0.5
   acc = 0.5
   rtde_frequency = 500.0
   dt = 1.0/rtde_frequency  # 2ms
   flags = RTDEControl.FLAG_VERBOSE | RTDEControl.FLAG_UPLOAD_SCRIPT
   ur_cap_port = 50002
   robot_ip = "localhost"

   lookahead_time = 0.1
   gain = 600

   # ur_rtde realtime priorities
   rt_receive_priority = 90
   rt_control_priority = 85

   rtde_r = RTDEReceive(robot_ip, rtde_frequency, [], True, False, rt_receive_priority)
   rtde_c = RTDEControl(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority)

   # Set application real-time priority
   os_used = sys.platform
   process = psutil.Process(os.getpid())
   if os_used == "win32":  # Windows (either 32-bit or 64-bit)
       process.nice(psutil.REALTIME_PRIORITY_CLASS)
   elif os_used == "linux":  # linux
       rt_app_priority = 80
       param = os.sched_param(rt_app_priority)
       try:
           os.sched_setscheduler(0, os.SCHED_FIFO, param)
       except OSError:
           print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
       else:
           print("Process real-time priority set to: %u" % rt_app_priority)

   time_counter = 0.0

   # Move to init position using moveL
   actual_tcp_pose = rtde_r.getActualTCPPose()
   init_pose = getCircleTarget(actual_tcp_pose, time_counter)
   rtde_c.moveL(init_pose, vel, acc)

   try:
       while True:
           t_start = rtde_c.initPeriod()
           servo_target = getCircleTarget(actual_tcp_pose, time_counter)
           rtde_c.servoL(servo_target, vel, acc, dt, lookahead_time, gain)
           rtde_c.waitPeriod(t_start)
           time_counter += dt

   except KeyboardInterrupt:
       print("Control Interrupted!")
       rtde_c.servoStop()
       rtde_c.stopScript()


Use with Dockerized UR Simulator
================================
See (https://github.com/urrsk/ursim_docker/blob/main/README.md for details)

first you need to clone the ursim_docker repository with:

.. code-block:: shell

    git clone https://github.com/urrsk/ursim_docker.git


Install docker
--------------
Next we install docker:

.. code-block:: shell

    sudo apt update
    sudo apt install docker.io
    sudo systemctl start docker
    sudo systemctl enable docker
    sudo systemctl status docker
    sudo usermod -aG docker $USER
    su - $USER


Build docker image
------------------
Then we build the docker image:

.. code-block:: shell

    docker build ursim/e-series -t myursim --build-arg VERSION=5.11.1.108318 --build-arg URSIM="https://s3-eu-west-1.amazonaws.com/ur-support-site/118926/URSim_Linux-5.11.1.108318.tar.gz"


Run docker image
----------------
Finally we run the docker image with:

.. code-block:: shell

     docker run --rm -it -p 5900:5900 -p 29999:29999 -p 30001-30004:30001-30004 myursim

.. _use-with-matlab:

Use with MATLAB
===============
MATLAB supports calling python library functions, please see
`this <https://se.mathworks.com/help/matlab/getting-started-with-python.html>`_ site for more information.

Here is an example of receiving the actual joint and tcp pose from the robot, and moving the robot
to some pre-defined cartesian position in MATLAB:

.. code-block:: matlab

    import py.rtde_receive.RTDEReceiveInterface
    import py.rtde_control.RTDEControlInterface

    rtde_r = RTDEReceiveInterface("localhost");
    rtde_c = RTDEControlInterface("localhost");

    actual_q = rtde_r.getActualQ();
    actual_tcp_pose = rtde_r.getActualTCPPose();

    % Convert to MATLAB array of double
    actual_q_array = cellfun(@double, cell(actual_q));
    actual_tcp_pose_array = cellfun(@double, cell(actual_tcp_pose));

    actual_q_array
    actual_tcp_pose_array

    position1 = [-0.343, -0.435, 0.50, -0.001, 3.12, 0.04];
    position2 = [-0.243, -0.335, 0.20, -0.001, 3.12, 0.04];

    rtde_c.moveL(position1);
    rtde_c.moveL(position2);
    rtde_c.stopRobot();
    clear

.. warning::
    Please notice, it is very important to include the 'clear' command and the end of execution, otherwise the ur_rtde
    threads will continue run in the background and you would not be able to execute the code again until the environment
    has been cleared.

.. note::
    Currently using the ur_rtde interface has only been tested with MATLAB R2019b using Python 2.7, since this seems
    to be the default interpreter of MATLAB R2019b. However, it should also work with Python 3.x


.. _use-with-robotiq-gripper:

Use with Robotiq Gripper
========================
There are currently 3 ways of using a Robotiq gripper with ur_rtde:

* **Option 1**: (Sending the robotiq preamble + function to be executed)

You can send the robotiq preamble script together with the function you want to run, using the
sendCustomScriptFunction() of the rtde_control interface. Unfortunately you have to send the preamble with
the gripper api functions everytime, which does give a bit of delay. You can download the preamble for
use with Python here: `robotiq_preamble.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_preamble.py>`_,
and a python interface for using the robotiq gripper this way here:
`robotiq_gripper_control.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_gripper_control.py>`_.

Example of this method:

.. code-block:: python

    from robotiq_gripper_control import RobotiqGripper
    from rtde_control import RTDEControlInterface
    import time

    rtde_c = RTDEControlInterface("<ROBOT_IP>")
    gripper = RobotiqGripper(rtde_c)

    # Activate the gripper and initialize force and speed
    gripper.activate()  # returns to previous position after activation
    gripper.set_force(50)  # from 0 to 100 %
    gripper.set_speed(100)  # from 0 to 100 %

    # Perform some gripper actions
    gripper.open()
    gripper.close()
    time.sleep(1)
    gripper.open()
    gripper.move(10)  # mm

    # Stop the rtde control script
    rtde_c.stopRobot()

.. admonition:: Pros
  :class: tip

    * Does not require any UR Cap to be installed.

.. admonition:: Cons
  :class: error

    * Slow execution, since the preamble is transmitted each time.
    * Simultaneous robot movements is not possible (since the rtde_control script is interrupted)

* **Option 2**: (Using the RS485 UR Cap)

Download the RS485 UR cap from here
`rs485-1.0.urcap <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/raw/master/ur_robot_driver/resources/rs485-1.0.urcap>`_,
install it on the robot and remember to remove the Robotiq_Grippers UR Cap as
these two cannot function together. It does not work with the Robotiq_Grippers UR Cap since this cap occupies the
RS485 port all the time.

You can then use the tool_communication script for making the robotiq serial port
available on your desktop. (eg. /tmp/ttyUR). Finally use a modbus RTU based driver to communicate through the serial
port. Alternatively you can avoid running the tool_communication script and just communicate directly to the socket at
the port specified in the RS485 cap (default is *54321*).

.. admonition:: Pros
  :class: tip

    * Allows you to communicate to the RS485 port on the robot.
    * This approach can be used with different grippers, that uses the UR RS485 connection.
    * Fast communication.

.. admonition:: Cons
  :class: error

    * Does not work together with the official Robotiq_Grippers UR Cap.
    * Requires you to install a UR Cap.

* **Option 3**: (Communicating directly with Robotiq_grippers UR Cap port)

A robotiq gripper can be controlled through a port (*63352*) that is opened by the Robotiq_grippers UR Cap. This
port provides direct communication to the gripper. So you simply connect to the robot IP at this port and you
can command it using the Robotiq string commands, see the 'Control' section of this
`manual <https://assets.robotiq.com/website-assets/support_documents/document/Hand-E_Manual_UniversalRobots_PDF_20191219.pdf>`_.

*C++*:

ur_rtde includes a C++ interface for robotiq grippers implemented by (Uwe Kindler). See the API here:
:ref:`Robotiq Gripper API <robotiq-gripper-api>`, and the example here: :ref:`Robotiq Gripper Example <robotiq-gripper-example>`

*Python*:

You can download an example Python class for controlling the gripper using this method here: `robotiq_gripper.py <https://sdurobotics.gitlab.io/ur_rtde/_static/robotiq_gripper.py>`_.
This class was implemented by Sam (Rasp) thanks! The class can be used like this:

.. code-block:: python

    import robotiq_gripper
    import time

    ip = "127.0.0.1"

    def log_info(gripper):
        print(f"Pos: {str(gripper.get_current_position()): >3}  "
              f"Open: {gripper.is_open(): <2}  "
              f"Closed: {gripper.is_closed(): <2}  ")

    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ip, 63352)
    print("Activating gripper...")
    gripper.activate()

    print("Testing gripper...")
    gripper.move_and_wait_for_pos(255, 255, 255)
    log_info(gripper)
    gripper.move_and_wait_for_pos(0, 255, 255)
    log_info(gripper)


.. admonition:: Pros
  :class: tip

    * Works with Robotiq_grippers UR Cap.
    * Fast communication.

.. admonition:: Cons
  :class: error

    * You might not be able to leverage existing robotiq drivers, depending on implementation.

My current recommendation is to use **Option 3** for controlling a Robotiq gripper, and if that does not suit your needs
go with **Option 2**. **Option 1** should only be used as a last resort.
