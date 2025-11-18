.PHONY: default bootstrap build build-force run run-rocker save clean check-commit

IMAGE_NAME = autosdv
CONTAINER_NAME = $(IMAGE_NAME)-container
# Get the current git branch name (fallback to commit hash if detached)
BRANCH_NAME = $(shell cd .. && git symbolic-ref --short HEAD 2>/dev/null || git rev-parse --short HEAD)
# Get the current git commit hash
COMMIT_HASH = $(shell cd .. && git rev-parse HEAD)
# Get the short version (prefix) of the commit hash
SHORT_HASH = $(shell cd .. && git rev-parse --short HEAD)
# Get the remote repository URL from the branch's upstream, or misuhsieh remote, or origin as fallback
REPO_URL = $(shell cd .. && git config --get branch.$(BRANCH_NAME).remote 2>/dev/null | xargs -I {} git config --get remote.{}.url || git config --get remote.misuhsieh.url || git config --get remote.origin.url)

# Check Docker availability and set DOCKER_CMD.
DOCKER_CMD := $(shell \
	if docker ps >/dev/null 2>&1; then \
		echo "docker"; \
	elif sudo -n docker ps >/dev/null 2>&1; then \
		echo "sudo docker"; \
	else \
		echo "ERROR"; \
	fi)

# Check if the command detection failed and stop if it did.
ifeq ($(DOCKER_CMD),ERROR)
	$(error "ERROR: Docker not installed or inaccessible. Install Docker and ensure user has access (e.g., add to 'docker' group).")
endif

# Inform the user which command is being used.
$(info Using $(DOCKER_CMD))

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
	$(DOCKER_CMD) run --privileged --rm tonistiigi/binfmt --install all
	sudo systemctl restart docker.service
#	$(DOCKER_CMD) buildx create --name autosdv-builder --use || true
#	@# If builder exists, use it; otherwise create it
	@if $(DOCKER_CMD) buildx inspect autosdv-builder >/dev/null 2>&1; then \
		echo "Using existing buildx builder: autosdv-builder"; \
		$(DOCKER_CMD) buildx use autosdv-builder || true; \
	else \
		echo "Creating new buildx builder: autosdv-builder"; \
		$(DOCKER_CMD) buildx create --name autosdv-builder --use || true; \
	fi


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
	@echo "Repository: $(REPO_URL)"
	@echo "Branch/Ref: $(BRANCH_NAME)"
	$(DOCKER_CMD) buildx build --load . \
		--network=host \
		--no-cache \
		--platform linux/arm64 \
		--build-arg COMMIT_HASH=$(BRANCH_NAME) \
		--build-arg REPO_URL=$(REPO_URL) \
		--tag $(IMAGE_NAME):$(SHORT_HASH) \
		--tag $(IMAGE_NAME):$(COMMIT_HASH)
	@echo "Built image $(IMAGE_NAME):$(SHORT_HASH) and $(IMAGE_NAME):$(COMMIT_HASH)"


# Added by Vincent
# Force build without cache
build-force: check-commit
	@echo "FORCE building Docker image (no cache) with commit hash: $(COMMIT_HASH) (short: $(SHORT_HASH))"
	@echo "Repository: $(REPO_URL)"
	@echo "Branch/Ref: $(BRANCH_NAME)"
	$(DOCKER_CMD) buildx build --load . \
		--network=host \
		--no-cache \
		--platform linux/arm64 \
		--build-arg COMMIT_HASH=$(BRANCH_NAME) \
		--build-arg REPO_URL=$(REPO_URL) \
		--tag $(IMAGE_NAME):$(SHORT_HASH) \
		--tag $(IMAGE_NAME):$(COMMIT_HASH)
	@echo "Built image $(IMAGE_NAME):$(SHORT_HASH) and $(IMAGE_NAME):$(COMMIT_HASH)"


# Auto-detect which image to use (local build or DockerHub)
DOCKERHUB_IMAGE := misuhsieh001/autosdv:2025.02-latest
IMAGE_TO_USE := $(shell \
	if $(DOCKER_CMD) image inspect $(IMAGE_NAME):$(SHORT_HASH) >/dev/null 2>&1; then \
		echo "$(IMAGE_NAME):$(SHORT_HASH)"; \
	elif $(DOCKER_CMD) image inspect $(DOCKERHUB_IMAGE) >/dev/null 2>&1; then \
		echo "$(DOCKERHUB_IMAGE)"; \
	else \
		echo ""; \
	fi)

# Auto-detect GPU runtime flags (Jetson uses --runtime=nvidia, others use --gpus all)
GPU_FLAGS := $(shell \
	if [ -f /etc/nv_tegra_release ] || uname -m | grep -q aarch64; then \
		echo "--runtime=nvidia"; \
	else \
		echo "--gpus all"; \
	fi)

# Run the container, creating it if it doesn't exist, or starting/attaching if it does.
run:
	@if [ -z "$(IMAGE_TO_USE)" ]; then \
		echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; \
		echo "❌ ERROR: No Docker image found!"; \
		echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; \
		echo ""; \
		echo "You need to either:"; \
		echo "  1. Build locally:  make build"; \
		echo "  2. Pull from DockerHub:  $(DOCKER_CMD) pull $(DOCKERHUB_IMAGE)"; \
		echo ""; \
		exit 1; \
	fi; \
	echo "Using image: $(IMAGE_TO_USE)"; \
	echo "Using GPU flags: $(GPU_FLAGS)"; \
	if ! $(DOCKER_CMD) ps -a --format '{{.Names}}' | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "Creating and running new container named $(CONTAINER_NAME)..."; \
		$(DOCKER_CMD) run -it --name $(CONTAINER_NAME) \
			$(GPU_FLAGS) \
			--net host \
			--privileged \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v $(HOME)/.Xauthority:/root/.Xauthority:rw \
			-v $(CURDIR)/..:/AutoSDV:rw \
			-v /dev:/dev:rw \
			-e DISPLAY=$(DISPLAY) \
			-e QT_X11_NO_MITSHM=1 \
			$(IMAGE_TO_USE) /bin/bash; \
	elif ! $(DOCKER_CMD) ps --format '{{.Names}}' | grep -q "^$(CONTAINER_NAME)$$"; then \
		echo "Starting and attaching to existing container $(CONTAINER_NAME)..."; \
		$(DOCKER_CMD) start -ai $(CONTAINER_NAME); \
	else \
		echo "Attaching to running container $(CONTAINER_NAME)..."; \
		$(DOCKER_CMD) exec -it $(CONTAINER_NAME) /bin/bash; \
	fi

# Alternative run command using rocker if available
run-rocker:
	rocker \
		--x11 \
		--nvidia \
		--user \
		--home \
		$(IMAGE_NAME):$(SHORT_HASH)

save:
	$(DOCKER_CMD) save $(IMAGE_NAME):$(SHORT_HASH) | zstd -T0 -o $(IMAGE_NAME)-$(SHORT_HASH).tar.zstd
	@echo "Saved image to $(IMAGE_NAME)-$(SHORT_HASH).tar.zstd"

clean:
	$(DOCKER_CMD) rmi $(IMAGE_NAME):$(SHORT_HASH) $(IMAGE_NAME):$(COMMIT_HASH) || true
