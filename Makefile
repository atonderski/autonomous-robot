
amd64:
	docker build -t autonomous-robot:amd64 -f Dockerfile.amd64 .


run: amd64
	docker-compose up


publish:
	docker build -t atonderski/autonomous-robot -f Dockerfile.armhf .
	docker push

