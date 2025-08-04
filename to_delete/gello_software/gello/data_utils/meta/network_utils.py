import random

def is_port_in_use(port: int) -> bool:
    import socket

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(("localhost", port)) == 0


def find_free_port(port):
    while is_port_in_use(port):
        port = (port + random.randint(1, 1000)) % 65536
    return port
