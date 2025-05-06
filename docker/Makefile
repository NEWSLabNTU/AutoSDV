.PHONY: default bootstrap build save

IMAGE_NAME = autosdv

default:
	@echo 'Usage:'
	@echo
	@echo 'make bootstrap'
	@echo '    Install dependent pakcages and configure the'
	@echo '    environment to build the container image.'
	@echo
	@echo 'make build'
	@echo '    Build the container image.'
	@echo
	@echo 'make run'
	@echo '    Enter the container shell.'
	@echo
	@echo 'make save'
	@echo '    Save a Docker image file.'

bootstrap:
	sudo apt-get install qemu binfmt-support qemu-user-static
	docker run --privileged --rm tonistiigi/binfmt --install all
	sudo systemctl restart docker.service

build:
	docker build . \
		-t $(IMAGE_NAME) \
		--platform linux/arm64

run:
	rocker \
		--x11 \
		--nvidia \
		--user \
		--home \
		$(IMAGE_NAME)

save: build
	docker save $(IMAGE_NAME) | zstd -T0 -o $(IMAGE_NAME).tar.zstd
