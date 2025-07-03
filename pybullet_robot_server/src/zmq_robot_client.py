import zmq
import pickle
import numpy as np
import time

def send_command(method, args=None, host="127.0.0.1", port=5556):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f"tcp://{host}:{port}")

    request = {"method": method}
    if args is not None:
        request["args"] = args

    socket.send(pickle.dumps(request))
    reply = socket.recv()
    result = pickle.loads(reply)
    return result

if __name__ == "__main__":
    # Example: get number of DOFs
    print("num_dofs:", send_command("num_dofs"))

    # Example: get current joint state
    print("joint_state:", send_command("get_joint_state"))

    # Example: get observations
    # # Set a random joint state for a 7 dof robot
    # target_joint_state =  np.random.uniform(-3.14, 3.14, size=7).tolist()
    # # the gripper has the limits [0, 1] i think
    # target_joint_state[-1] = (target_joint_state[-1] + 3.14) / 6.28 # normalize to [0, 1]

    # print("target_joint_state:", target_joint_state)
    # print("command_joint_state:", send_command("command_joint_state", {"joint_state": target_joint_state}))

    print('starting loop')

    try:
        while True:
            # Set a random joint state for a 7 dof robot
            target_joint_state =  np.random.uniform(-3.14, 3.14, size=7).tolist()
            # the gripper has the limits [0, 1] i think
            target_joint_state[-1] = (target_joint_state[-1] + 3.14) / 6.28 # normalize to [0, 1]

            print("target_joint_state:", target_joint_state)
            print("command_joint_state:", send_command("command_joint_state", {"joint_state": target_joint_state}))


            # get observations and display visualization
            obs = send_command("get_observations")
            print("observations keys:", obs.keys())

            print('hi')
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Stopped by user.")
    