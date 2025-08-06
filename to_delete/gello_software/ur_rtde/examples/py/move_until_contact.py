from rtde_control import RTDEControlInterface as RTDEControl

rtde_c = RTDEControl("127.0.0.1")
speed = [0, 0, -0.100, 0, 0, 0]
rtde_c.moveUntilContact(speed)

rtde_c.stopScript()
