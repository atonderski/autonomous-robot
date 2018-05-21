
amd:
	docker build -t autonomous-robot:amd64 -f Dockerfile.amd64 .
arm:
	docker build -t autonomous-robot:armhf -f Dockerfile.armhf .

detector-amd:
	cd detector; docker build -t detector:amd64 -f Dockerfile.amd64 .
detector-arm:
	cd detector; docker build -t detector:armhf -f Dockerfile.armhf .


run: amd
	docker-compose up --force-recreate


build-arm: arm detector-arm

build-amd: amd detector-amd

publish-arm:
	docker tag autonomous-robot:armhf atonderski/autonomous-robot:armhf
	docker tag detector:armhf atonderski/detector:armhf
	docker push atonderski/autonomous-robot:armhf
	docker push atonderski/detector:armhf

publish-amd:
	docker tag autonomous-robot:amd64 atonderski/autonomous-robot:amd64
	docker tag detector:amd64 atonderski/detector:amd64
	docker push atonderski/autonomous-robot:amd64
	docker push atonderski/detector:amd64
