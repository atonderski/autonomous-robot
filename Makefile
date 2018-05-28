
amd:
	docker build -t autonomous-robot:amd64 -f Dockerfile.amd64 .
arm:
	docker build -t autonomous-robot:armhf -f Dockerfile.armhf .

detector-amd:
	cd detector; docker build -t detector:amd64 -f Dockerfile.amd64 .
detector-arm:
	cd detector; docker build -t detector:armhf -f Dockerfile.armhf .

opencv-amd:
	cd detector; docker build -t opencv:amd64 -f Dockerfile.base.amd64 .
opencv-arm:
	cd detector; docker build -t opencv:armhf -f Dockerfile.base.armhf .

run: amd
	docker-compose up --force-recreate


build-arm: arm detector-arm

build-amd: amd detector-amd

publish-detector-arm:
	docker tag detector:armhf atonderski/detector:armhf
	docker push atonderski/detector:armhf
publish-detector-amd:
	docker tag detector:amd64 atonderski/detector:amd64
	docker push atonderski/detector:amd64

publish-opencv-arm:
	docker tag opencv:armhf atonderski/opencv:armhf
	docker push atonderski/opencv:armhf
publish-opencv-amd:
	docker tag opencv:amd64 atonderski/opencv:amd64
	docker push atonderski/opencv:amd64

publish-arm:
	docker tag autonomous-robot:armhf atonderski/autonomous-robot:armhf
	docker push atonderski/autonomous-robot:armhf
publish-amd:
	docker tag autonomous-robot:amd64 atonderski/autonomous-robot:amd64
	docker push atonderski/autonomous-robot:amd64
