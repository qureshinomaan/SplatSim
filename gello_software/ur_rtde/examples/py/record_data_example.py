from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import argparse
import sys


def parse_args(args):
    """Parse command line parameters

    Args:
      args ([str]): command line parameters as list of strings

    Returns:
      :obj:`argparse.Namespace`: command line parameters namespace
    """
    parser = argparse.ArgumentParser(
        description="Record data example")
    parser.add_argument(
        "-ip",
        "--robot_ip",
        dest="ip",
        help="IP address of the UR robot",
        type=str,
        default='localhost',
        metavar="<IP address of the UR robot>")
    parser.add_argument(
        "-o",
        "--output",
        dest="output",
        help="data output (.csv) file to write to (default is \"robot_data.csv\"",
        type=str,
        default="robot_data.csv",
        metavar="<data output file>")
    parser.add_argument(
        "-f",
        "--frequency",
        dest="frequency",
        help="the frequency at which the data is recorded (default is 500Hz)",
        type=float,
        default=500.0,
        metavar="<frequency>")

    return parser.parse_args(args)


def main(args):
    """Main entry point allowing external calls

    Args:
      args ([str]): command line parameter list
    """
    args = parse_args(args)
    dt = 1 / args.frequency
    rtde_r = RTDEReceive(args.ip, args.frequency)
    rtde_r.startFileRecording(args.output)
    print("Data recording started, press [Ctrl-C] to end recording.")
    i = 0
    try:
        while True:
            t_start = rtde_r.initPeriod()
            start = time.time()
            if i % 10 == 0:
                sys.stdout.write("\r")
                sys.stdout.write("{:3d} samples.".format(i))
                sys.stdout.flush()
            rtde_r.waitPeriod(t_start)
            i += 1

    except KeyboardInterrupt:
        rtde_r.stopFileRecording()
        print("\nData recording stopped.")


if __name__ == "__main__":
    main(sys.argv[1:])
