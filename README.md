# autonomous-robot

How to build:

    docker build -t myname -f Dockerfile.amd64 .

To building for arm instead, just replace Dockerfile.amd64 with Dockerfile.armhf:

    docker build -t myname -f Dockerfile.armhf .

How to run:

    docker run -it --net=host myname ./helloworld

or:

    docker run -it --net=host myname ./autonomous-robot

