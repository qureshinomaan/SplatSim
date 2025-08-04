import os
UUID_DICT = {
    # Get by running nvidia-smi -L on each machine
    # Needed for SLURM mappings
}

def get_hostname():
    import socket
    hostname = socket.gethostname()
    return hostname

def get_gpu_uuid():
    import os
    gpu_info_list = os.popen('nvidia-smi -L').read().split('\n')
    uuids = []
    for gpu_info in gpu_info_list:
        if 'UUID' in gpu_info:
            gpu_uuid = gpu_info.split('UUID:')[1].strip()
            gpu_uuid = gpu_uuid.rstrip(')')
            uuids.append(gpu_uuid)
    return uuids

def get_graphics_gpu_ids():
    uuids = get_gpu_uuid()
    # If CUDA_VISIBLE_DEVICES is set, use it
    if 'CUDA_VISIBLE_DEVICES' in os.environ:
        gpu_ids = [int(i) for i in os.environ['CUDA_VISIBLE_DEVICES'].split(',')]
        uuids = [uuids[i] for i in gpu_ids]

    hostname = get_hostname()
    gpu_ids = [UUID_DICT[hostname][uuid] for uuid in uuids]
    return gpu_ids

def vulkan_cuda_idxes(mode, num_gpus):
    if mode == 'slurm':
        from gpu_info import get_graphics_gpu_ids
        vulkan_gpu_idxes = get_graphics_gpu_ids()[:num_gpus]
        cuda_gpu_idxes = list(range(num_gpus))
        print(f"CUDA GPUs: {cuda_gpu_idxes}")
        print(f"Vulkan GPUs: {vulkan_gpu_idxes}")
    elif mode == "basic":
        vulkan_gpu_idxes = [int(x) for x in os.environ["WHICH_GPUS"].split(",")][:num_gpus]
        cuda_gpu_idxes = vulkan_gpu_idxes
    else:
        raise NotImplementedError(f"Unsupported launcher: {os.environ['HYDRA_LAUNCHER']}")
    
    return cuda_gpu_idxes, vulkan_gpu_idxes

if __name__ == '__main__':
    g_ids = get_graphics_gpu_ids()
    print(g_ids)
    
