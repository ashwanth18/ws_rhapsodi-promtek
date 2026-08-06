# Discoverable developer entry points for the cell-layout / scooping loop.
# Run `make` (or `make help`) to list tasks. Scripts under scripts/ hold the logic.
# Sequence docs: docs/CELL_LAYOUT_DEV_LOOP.md

.DEFAULT_GOAL := help

.PHONY: help author bench sim arm-session lexium-session export-poses status \
        validate-layouts layout-parity touch-off-parity gen-vscode-tasks \
        check-vscode-tasks api-sandbox-up api-sandbox-down dashboard-dev \
        console-dev deploy-jetson push-layout test-backend ros-image-local

LAYOUT ?= dual-container
ROBOT ?= niryo
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
ROS_IMAGE_LOCAL_TAG ?= iserenity/rhapsodi-promtek:ros-prod-lexium
BUILDX_BUILDER ?= multiarch

##@ Authoring
help:  ## List available tasks (grouped)
	@awk 'BEGIN {FS = ":.*##"; printf "\nUsage:\n  make \033[36m<target>\033[0m LAYOUT=$(LAYOUT) ROBOT=$(ROBOT)\n"} \
	  /^##@/ {printf "\n\033[1m%s\033[0m\n", substr($$0, 5)} \
	  /^[a-zA-Z0-9_-]+:.*?##/ {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}' \
	  $(MAKEFILE_LIST)
	@echo
	@echo "  Override: make author LAYOUT=lightsout-single-vessel ROBOT=jaka"
	@echo "  Sequence: docs/CELL_LAYOUT_DEV_LOOP.md"

author:  ## Bench + interactive cell-layout editor (LAYOUT=<id> ROBOT=<id>)
	$(ROOT)/scripts/dev_author.sh $(LAYOUT) $(ROBOT)

bench:  ## Laptop bench loop, mock hardware, isolated ROS domain (LAYOUT=<id> ROBOT=<id>)
	$(ROOT)/scripts/dev_bench.sh $(LAYOUT) $(ROBOT)

sim:  ## Gazebo sim with layout-composed world (ROBOT=<id> LAYOUT=<id>)
	$(ROOT)/scripts/dev_sim.sh $(ROBOT) $(LAYOUT)

arm-session:  ## Real-arm authoring handoff (LAYOUT=<id>); stops Pi scooping_stack. No motion commanded.
	$(ROOT)/scripts/dev_arm_session.sh $(LAYOUT) $(ROBOT)

lexium-session:  ## Native RViz + Lexium Safety panel against laptop scooping_stack (ROBOT=jaka)
	$(ROOT)/scripts/dev_lexium_session.sh $(ROBOT)

export-poses:  ## Save timestamped scoop pose set under layouts/<id>/poses/sets/ (default intact)
	$(ROOT)/scripts/export_scoop_poses.sh

status:  ## Doctor: active layout, dirty tree, pose freshness, validation, next step
	$(ROOT)/scripts/dev_status.sh

##@ Images
ros-image-local:  ## Build ros-prod for local amd64 (Lexium packages); tag ROS_IMAGE_LOCAL_TAG
	docker buildx build --builder $(BUILDX_BUILDER) --platform linux/amd64 \
		-f $(ROOT)/docker/ros/Dockerfile.prod \
		-t $(ROS_IMAGE_LOCAL_TAG) --load $(ROOT)

##@ Validate
validate-layouts:  ## Schema + catalog validate every config/layouts/*.yaml
	python3 $(ROOT)/scripts/validate_layouts.py

layout-parity:  ## Assert generated Gazebo worlds contain every enabled layout object
	python3 $(ROOT)/scripts/test_layout_sim_parity.py

touch-off-parity:  ## Assert C++ touch-off fit matches calibrate_container_pose.py
	python3 $(ROOT)/scripts/test_touch_off_parity.py

gen-vscode-tasks:  ## Rewrite .vscode/tasks.json layout/robot pickers from config
	python3 $(ROOT)/scripts/gen_vscode_tasks.py --write

check-vscode-tasks:  ## CI: fail if .vscode/tasks.json pickers are stale
	python3 $(ROOT)/scripts/gen_vscode_tasks.py --check

##@ Web
api-sandbox-up:  ## Start local mode/API sandbox only (docker-compose.sim.yml; no Gazebo)
	docker compose -f $(ROOT)/docker-compose.sim.yml up -d --build

api-sandbox-down:  ## Stop local mode/API sandbox
	docker compose -f $(ROOT)/docker-compose.sim.yml down

dashboard-dev:  ## Robot dashboard Vite dev server
	cd $(ROOT)/src/dashboard && npm run dev

console-dev:  ## Fleet Console API + web (two terminals recommended; starts API)
	cd $(ROOT)/src/fleet_console && uvicorn app.main:app --reload --port 8090

##@ Deploy
deploy-jetson:  ## Redeploy Fleet Console to Jetson (LOCAL_SYNC=1 for dirty tree)
	bash $(ROOT)/scripts/deploy_jetson_fleet_console.sh

push-layout:  ## DEV FAST-PATH: rsync layout YAML to Pi + apply (bypasses release provenance)
	@echo "WARNING: push-layout bypasses release provenance; prefer Fleet Console apply after commit/release." >&2
	$(ROOT)/scripts/push_layout.sh $(LAYOUT)

##@ Test
test-backend:  ## Run backend unit tests (pytest)
	cd $(ROOT)/src/backend && python3 -m pytest app/tests -q
