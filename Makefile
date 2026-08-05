# Discoverable developer entry points for the cell-layout / scooping loop.
# Run `make` (or `make help`) to list tasks. Scripts under scripts/ hold the logic.

.DEFAULT_GOAL := help

.PHONY: help bench arm-session export-poses validate-layouts layout-parity \
        sim-up sim-down dashboard-dev console-dev deploy-jetson push-layout \
        test-backend

LAYOUT ?= dual-container
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

help:  ## List available tasks
	@grep -hE '^[a-zA-Z0-9_-]+:.*?##' $(MAKEFILE_LIST) \
	  | awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-20s\033[0m %s\n",$$1,$$2}'
	@echo
	@echo "  LAYOUT=$(LAYOUT)  (override: make bench LAYOUT=lightsout-single-vessel)"

bench:  ## Laptop bench loop, mock hardware, isolated ROS domain (LAYOUT=<id>)
	$(ROOT)/scripts/dev_bench.sh $(LAYOUT)

arm-session:  ## Real-arm authoring handoff; stops Pi scooping_stack. No motion commanded.
	$(ROOT)/scripts/dev_arm_session.sh

export-poses:  ## Export authored scoop poses into config/layouts/<id>/poses.yaml
	$(ROOT)/scripts/export_scoop_poses.sh

validate-layouts:  ## Schema-validate every config/layouts/*.yaml
	python3 $(ROOT)/scripts/validate_layouts.py

layout-parity:  ## Check sim/real parity of generated Gazebo world includes
	python3 $(ROOT)/scripts/test_layout_sim_parity.py

sim-up:  ## Start local sim stack (docker-compose.sim.yml)
	docker compose -f $(ROOT)/docker-compose.sim.yml up -d --build

sim-down:  ## Stop local sim stack
	docker compose -f $(ROOT)/docker-compose.sim.yml down

dashboard-dev:  ## Robot dashboard Vite dev server
	cd $(ROOT)/src/dashboard && npm run dev

console-dev:  ## Fleet Console API + web (two terminals recommended; starts API)
	cd $(ROOT)/src/fleet_console && uvicorn app.main:app --reload --port 8090

deploy-jetson:  ## Redeploy Fleet Console to Jetson (LOCAL_SYNC=1 for dirty tree)
	bash $(ROOT)/scripts/deploy_jetson_fleet_console.sh

push-layout:  ## DEV FAST-PATH: rsync layout YAML to Pi + apply (bypasses release provenance)
	@echo "WARNING: push-layout bypasses release provenance; prefer Fleet Console apply after commit/release." >&2
	$(ROOT)/scripts/push_layout.sh $(LAYOUT)

test-backend:  ## Run backend unit tests (pytest)
	cd $(ROOT)/src/backend && python3 -m pytest app/tests -q
