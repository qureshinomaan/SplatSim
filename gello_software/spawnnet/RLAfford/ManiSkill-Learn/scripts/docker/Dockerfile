FROM nvidia/cudagl:11.0-devel-ubuntu20.04
RUN apt update
RUN apt install -y unzip htop rsync tmux ssh git curl nano feh
RUN mkdir -p /usr/share/vulkan/icd.d
RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libGLX_nvidia.so.0", "api_version": "1.1.84"}}' > /usr/share/vulkan/icd.d/nvidia_icd.json
RUN curl -O https://repo.anaconda.com/miniconda/Miniconda3-py39_4.9.2-Linux-x86_64.sh && bash Miniconda3-py39_4.9.2-Linux-x86_64.sh -b -p && rm Miniconda3-py39_4.9.2-Linux-x86_64.sh
RUN cd /root && git clone https://github.com/haosulab/ManiSkill.git
RUN cd /root && git clone https://github.com/haosulab/ManiSkill-Learn.git
RUN ls /root
RUN /root/miniconda3/bin/conda init
RUN /root/miniconda3/bin/conda env create -f /root/ManiSkill/environment.yml
RUN cd /root/ManiSkill-Learn && . "/root/miniconda3/etc/profile.d/conda.sh" && conda activate mani_skill && pip install -e .
RUN cd /root/ManiSkill && . "/root/miniconda3/etc/profile.d/conda.sh" && conda activate mani_skill && pip install -e .
