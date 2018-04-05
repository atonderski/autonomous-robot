

publish:
	docker build -t atonderski/autonomous-robot -f Dockerfile.armhf .
	docker push

