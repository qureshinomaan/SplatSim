from dataclasses import dataclass
from pathlib import Path

import tyro

from gello.robots.robot import BimanualRobot, PrintRobot
from gello.zmq_core.robot_node import ZMQServerRobot


@dataclass
class Args:
    robot: str = "xarm"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.1.10"
    gaussian_path : str = "/home/jennyw2/data/output/robot_iphone/point_cloud/iteration_30000/point_cloud.ply"


def launch_robot_server(args: Args):
    port = args.robot_port
    if args.robot == "sim_ur":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        # gripper_xml = None
        from gello.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_push":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        # gripper_xml = None
        # from gello.robots.sim_robot_pybullet import PybulletRobotServer
        from gello.robots.sim_robot_pybullet_push import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname,
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_cup":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        # gripper_xml = None
        # from gello.robots.sim_robot_pybullet import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_cup import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_pick_planner import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_pick_place_planner import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_orange_on_plate import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_assembly import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_articulated import PybulletRobotServer
        from gello.robots.sim_robot_pybullet_deformable import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname,
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_orange":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot_pybullet_orange_on_plate import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname, serve_mode=PybulletRobotServer.SERVE_MODES.GENERATE_DEMOS,
           env_config_name="orange_on_plate",
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_orange_interactive":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot_pybullet_orange_on_plate import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname, serve_mode=PybulletRobotServer.SERVE_MODES.INTERACTIVE,
           env_config_name="orange_on_plate",
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_apple":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot_pybullet_orange_on_plate import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname, serve_mode=PybulletRobotServer.SERVE_MODES.GENERATE_DEMOS,
           env_config_name="apple_on_plate",
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_apple_interactive":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot_pybullet_orange_on_plate import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname, serve_mode=PybulletRobotServer.SERVE_MODES.INTERACTIVE,
           env_config_name="apple_on_plate",
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_banana":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot_pybullet_orange_on_plate import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname, serve_mode=PybulletRobotServer.SERVE_MODES.GENERATE_DEMOS,
           env_config_name="banana_on_plate",
        )
        server.serve()

    elif args.robot == "sim_ur_pybullet_banana_interactive":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from gello.robots.sim_robot_pybullet_orange_on_plate import PybulletRobotServer

        server = PybulletRobotServer(
           port=port, host=args.hostname, serve_mode=PybulletRobotServer.SERVE_MODES.INTERACTIVE,
           env_config_name="banana_on_plate",
        )
        server.serve()

    elif args.robot == "sim_ur_splat":
        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        # gripper_xml = None
        # from gello.robots.sim_robot_pybullet import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_splat_6DOF import PybulletRobotServer
        # from gello.robots.sim_robot_pybullet_splat_servoing import PybulletRobotServer
        from gello.robots.sim_robot_pybullet_splat_servoing_improved import GaussianRenderServer
        server = GaussianRenderServer(
           port=port, host=args.hostname, gaussian_path=args.gaussian_path
        )
        server.serve()

    elif args.robot == "sim_panda":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_xarm":
        from gello.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).parent.parent / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    else:
        if args.robot == "xarm":
            from gello.robots.xarm_robot import XArmRobot

            robot = XArmRobot(ip=args.robot_ip)
        elif args.robot == "ur":
            from gello.robots.ur import URRobot

            robot = URRobot(robot_ip=args.robot_ip, no_gripper=False)
        elif args.robot == "panda":
            from gello.robots.panda import PandaRobot

            robot = PandaRobot(robot_ip=args.robot_ip, no_gripper=False)
        elif args.robot == "bimanual_ur":
            from gello.robots.ur import URRobot

            # IP for the bimanual robot setup is hardcoded
            _robot_l = URRobot(robot_ip="192.168.2.11")
            _robot_r = URRobot(robot_ip="192.168.1.11")
            robot = BimanualRobot(_robot_l, _robot_r)
        
        elif args.robot == "ur_pybullet":
            from gello.robots.ur_pybullet import URRobotPybullet
            robot = URRobotPybullet(robot_ip=args.robot_ip, no_gripper=True)

        elif args.robot == "none" or args.robot == "print":
            robot = PrintRobot(8)

        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, xarm, ur, bimanual_ur, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
