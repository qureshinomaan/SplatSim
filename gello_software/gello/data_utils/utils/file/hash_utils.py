import numpy as np


def md5sum(filename, block_size=None):
    import hashlib

    if block_size is None:
        block_size = 65536
    hash_res = hashlib.md5()
    with open(filename, "rb") as f:
        for block in iter(lambda: f.read(block_size), b""):
            hash_res.update(block)
    return hash_res.hexdigest()


def check_md5sum(filename, md5, block_size=None):
    if not (isinstance(md5, str) and len(md5) == 32):
        raise ValueError(f"MD5 must be 32 chars: {md5}")
    md5_actual = md5sum(filename, block_size=block_size)
    if md5_actual == md5:
        return True
    else:
        print(f"MD5 does not match!: {filename} has md5 {md5_actual}, target md5 is {md5}")
        return False


def masked_crc(data: bytes) -> bytes:
    import struct

    try:
        from crc32c import crc32c
    except ImportError:
        print("Cannot import crc32c, please install it!")
        exit(0)
    """CRC checksum."""
    mask = 0xA282EAD8
    crc = crc32c(data)
    masked = ((crc >> 15) | (crc << 17)) + mask
    masked = np.uint32(masked & np.iinfo(np.uint32).max)
    masked_bytes = struct.pack("<I", masked)
    return masked_bytes
