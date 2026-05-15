.PHONY: bridge-up bridge-down test-sitl test-sitl-strict test-wasm-sitl test-wasm-sitl-strict

SITL_NAME ?= ardupilot-sitl
SITL_IMAGE ?= radarku/ardupilot-sitl:eff32c1f98152ac3d1dc09a1e475733b73ce569f
MAVKIT_SITL_TCP_HOST ?= 127.0.0.1
MAVKIT_SITL_TCP_PORT ?= 5760
PYTHON ?= python3

bridge-up:
	docker rm -f $(SITL_NAME) >/dev/null 2>&1 || true
	docker pull $(SITL_IMAGE)
	docker run -d --rm --name $(SITL_NAME) -p $(MAVKIT_SITL_TCP_HOST):$(MAVKIT_SITL_TCP_PORT):5760 \
			--entrypoint /ardupilot/build/sitl/bin/arducopter \
			$(SITL_IMAGE) \
			--model + --speedup 1 --defaults /ardupilot/Tools/autotest/default_params/copter.parm \
			--home 42.3898,-71.1476,14.0,270.0 -w
	$(PYTHON) scripts/wait_tcp.py $(MAVKIT_SITL_TCP_HOST) $(MAVKIT_SITL_TCP_PORT) 120
	@printf "SITL ready at tcp:%s:%s\n" "$(MAVKIT_SITL_TCP_HOST)" "$(MAVKIT_SITL_TCP_PORT)"

test-sitl:
	MAVKIT_SITL_IMAGE=$(SITL_IMAGE) $(PYTHON) scripts/run_sitl_tests.py

test-sitl-strict:
	MAVKIT_SITL_IMAGE=$(SITL_IMAGE) $(PYTHON) scripts/run_sitl_tests.py --strict

test-wasm-sitl:
	MAVKIT_SITL_IMAGE=$(SITL_IMAGE) $(PYTHON) scripts/run_wasm_sitl_tests.py

test-wasm-sitl-strict:
	MAVKIT_SITL_IMAGE=$(SITL_IMAGE) $(PYTHON) scripts/run_wasm_sitl_tests.py --strict

bridge-down:
	docker rm -f $(SITL_NAME) >/dev/null 2>&1 || true
