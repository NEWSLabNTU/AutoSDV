.PHONY: default bootstrap build run save clean

IMAGE_NAME = autosdv
# Get the current git commit hash
COMMIT_HASH = $(shell cd .. && git rev-parse HEAD)
# Get the short version (prefix) of the commit hash
SHORT_HASH = $(shell cd .. && git rev-parse --short HEAD)

default:
	@echo 'Usage:'
	@echo
	@echo 'make bootstrap'
	@echo '    Install dependent packages and configure the'
	@echo '    environment to build the container image.'
	@echo
	@echo 'make build'
	@echo '    Build the container image using the current commit hash ($(COMMIT_HASH)).'
	@echo '    Will tag the image as $(IMAGE_NAME):$(SHORT_HASH)'
	@echo
	@echo 'make build-force'
	@echo '    Force rebuild the container image without using cache using the current commit hash.'
	@echo
	@echo 'make run'
	@echo '    Enter the container shell.'
	@echo
	@echo 'make save'
	@echo '    Save a Docker image file.'
	@echo
	@echo 'make clean'
	@echo '    Remove the Docker image.'

bootstrap:
	sudo apt-get install qemu binfmt-support qemu-user-static
	docker run --privileged --rm tonistiigi/binfmt --install all
	sudo systemctl restart docker.service

# First check if the commit hash is available on the remote
check-commit:
	@if ! cd .. && git branch -r --contains $(COMMIT_HASH) | grep -q 'origin/'; then \
		echo "Warning: Commit $(COMMIT_HASH) does not appear to be pushed to the remote repository."; \
		echo "Please push your changes to the remote repository first:"; \
		echo "  git push origin HEAD"; \
		exit 1; \
	fi

# Build using the current commit hash
build: check-commit
	@echo "Building Docker image with commit hash: $(COMMIT_HASH) (short: $(SHORT_HASH))"
	docker build . \
		-t $(IMAGE_NAME):$(SHORT_HASH) \
		-t $(IMAGE_NAME):$(COMMIT_HASH) \
		--build-arg COMMIT_HASH=$(COMMIT_HASH) \
		--platform linux/arm64
	@echo "Built image $(IMAGE_NAME):$(SHORT_HASH) and $(IMAGE_NAME):$(COMMIT_HASH)"

run:
	docker run \
		-it \
		--rm \
		--platform linux/arm64 \
		-e DISPLAY=${DISPLAY} \
		--network=host \
		$(IMAGE_NAME):$(SHORT_HASH)

# Alternative run command using rocker if available
run-rocker:
	rocker \
		--x11 \
		--nvidia \
		--user \
		--home \
		$(IMAGE_NAME):$(SHORT_HASH)

save:
	docker save $(IMAGE_NAME):$(SHORT_HASH) | zstd -T0 -o $(IMAGE_NAME)-$(SHORT_HASH).tar.zstd
	@echo "Saved image to $(IMAGE_NAME)-$(SHORT_HASH).tar.zstd"

clean:
	docker rmi $(IMAGE_NAME):$(SHORT_HASH) $(IMAGE_NAME):$(COMMIT_HASH) || true
