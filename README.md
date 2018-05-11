# autonomous-robot

How to build:

    docker build -t myname -f Dockerfile.amd64 .

To building for arm instead, just replace Dockerfile.amd64 with Dockerfile.armhf:

    docker build -t myname -f Dockerfile.armhf .

How to run:

    docker run -it --net=host myname ./helloworld

or:

    docker run -it --net=host myname ./autonomous-robot

Subsumer-controller has a separate config-file that can set most of the constants used by the AFSM derivatives and the acceleration regulator. The configuration file gets packed into the docker image which makes the editing a bit bothersome. The following commands will be most useful:

    docker run -d <image name> --flags

    docker exec -it <container ID> /bin/ash

Inside the container, the configuration file can be edited with vi. Once edited, exit container and commit the changes to the image:

    docker commit <container ID> <image name>

Kill the container and run the newest image. The changes should take effect. It may also be possible to skip committing the changes and simply restarting the container with:

    docker restart <container ID>
