docker build . < Dockerfile -t maniskill-2021
# docker rm -f  maniskill # Delete the container
docker run --runtime=nvidia -e DISPLAY=:0 -itd -v /tmp/.X11-unix:/tmp/.X11-unix --rm --name maniskill maniskill-2021:latest /bin/bash -c "while true; do echo hello world; sleep 1; done"
docker exec -it maniskill bash
