ARTIFACTS_DIR ?= artifacts
BAG_TOPICS ?= /drone/odom /sim/ground_truth/odom /tf /sim/scenario_markers
OPTIONAL_BAG_TOPICS ?= /sim/camera/front/image_raw
DURATION ?= 10
LOG_FLAGS ?= -f
TOPIC ?= /drone/odom

.PHONY: help preflight build up up-baseline baseline-heartbeat up-mujoco probe-ros-visualization probe-camera-image topic-list topic-hz bag-record artifact-dirs test-json-stub test-mujoco-quad test-frame-conventions test-ros-frame-conventions up-viz down logs shell-ros2 shell-sitl shell-physics test test-e2e

help:
	@printf '%s\n' \
		'Targets:' \
		'  help           Show this help' \
		'  preflight      Check Docker Desktop for Mac host availability' \
		'  build          Build the shared simulation image' \
		'  up             Start headless placeholder services' \
		'  up-baseline    Start baseline ArduCopter SITL and ROS 2 receiver' \
		'  baseline-heartbeat  Wait for the baseline MAVLink heartbeat' \
		'  up-mujoco      Start ArduCopter SITL with MuJoCo JSON physics' \
		'  probe-ros-visualization  Wait for DFD-010 ROS visualization topics' \
		'  probe-camera-image  Probe DFD-011 MuJoCo camera image output' \
		'  topic-list     List ROS 2 topics and tee output to artifacts/test-logs' \
		'  topic-hz       Measure a ROS 2 topic rate, e.g. make topic-hz TOPIC=/drone/odom' \
		'  bag-record     Record DFD-015 ROS bag artifacts, e.g. make bag-record DURATION=10' \
		'  test-json-stub Run JSON protocol stub unit tests in Docker' \
		'  test-mujoco-quad  Run the minimal MuJoCo quad self-test in Docker' \
		'  test-frame-conventions  Run ENU/NED and altitude sign unit tests' \
		'  test-ros-frame-conventions  Run ROS odometry frame-convention unit tests' \
		'  up-viz         Start MuJoCo, ROS 2, and RViz2/noVNC on localhost:6080' \
		'  down           Stop and remove Compose services' \
		'  logs           Follow Compose service logs and tee output to artifacts/test-logs' \
		'  shell-ros2     Open a shell in the ros2 service image' \
		'  shell-sitl     Open a shell in the sitl service image' \
		'  shell-physics  Open a shell in the physics service image' \
		'  test           Run scaffold smoke checks in Docker' \
		'  test-e2e       Run full MuJoCo/SITL/ROS 2 takeoff E2E smoke test'

preflight:
	@sh scripts/preflight.sh

artifact-dirs:
	mkdir -p $(ARTIFACTS_DIR)/ardupilot $(ARTIFACTS_DIR)/rosbags $(ARTIFACTS_DIR)/test-logs

build:
	docker compose build

up:
	docker compose up -d sitl physics ros2

up-baseline:
	docker compose --profile baseline up sitl-baseline ros2

baseline-heartbeat:
	docker compose exec ros2 python3 scripts/mavlink_wait_heartbeat.py --connect udp:0.0.0.0:14550

up-mujoco:
	docker compose --profile mujoco up sitl-json physics ros2

probe-ros-visualization:
	docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/ros_topic_probe.py /sim/ground_truth/odom /tf /sim/scenario_markers'

probe-camera-image:
	docker compose exec ros2 bash -lc 'source /workspace/install/setup.bash && python3 scripts/image_probe.py /sim/camera/front/image_raw'

topic-list: artifact-dirs
	docker compose exec -T ros2 bash -lc 'set -eo pipefail; source /opt/ros/humble/setup.bash; if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi; mkdir -p /workspace/artifacts/test-logs; log=/workspace/artifacts/test-logs/topic-list-$$(date -u +%Y%m%dT%H%M%SZ).log; python3 scripts/ros_topic_list_probe.py --required $(BAG_TOPICS) --optional $(OPTIONAL_BAG_TOPICS) | tee "$$log"; printf "topic-list-log=%s\n" "$$log"'

topic-hz: artifact-dirs
	docker compose exec -T ros2 bash -lc 'set -eo pipefail; source /opt/ros/humble/setup.bash; if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi; mkdir -p /workspace/artifacts/test-logs; log=/workspace/artifacts/test-logs/topic-hz-$$(date -u +%Y%m%dT%H%M%SZ).log; python3 scripts/ros_topic_hz_probe.py "$(TOPIC)" --duration "$(DURATION)" | tee "$$log"; printf "topic-hz-log=%s\n" "$$log"'

bag-record: artifact-dirs
	docker compose exec -T ros2 bash -lc 'set -eo pipefail; source /opt/ros/humble/setup.bash; if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi; mkdir -p /workspace/artifacts/test-logs; export DURATION="$(DURATION)"; export BAG_TOPICS="$(BAG_TOPICS)"; export OPTIONAL_BAG_TOPICS="$(OPTIONAL_BAG_TOPICS)"; log=/workspace/artifacts/test-logs/bag-record-$$(date -u +%Y%m%dT%H%M%SZ).log; scripts/record_ros_bag.sh | tee "$$log"; printf "bag-record-log=%s\n" "$$log"'

test-json-stub:
	docker compose run --rm physics python3 -m pytest tests/test_ardupilot_json_stub.py

test-mujoco-quad:
	docker compose run --rm physics python3 -m src.physics.self_test

test-frame-conventions:
	docker compose run --rm physics pytest tests/test_frame_conventions.py

test-ros-frame-conventions:
	docker compose run --rm ros2 pytest tests/test_ros_frame_conventions.py

up-viz:
	docker compose --profile mujoco --profile viz up -d sitl-json physics ros2 viz

down:
	docker compose --profile baseline --profile mujoco --profile viz down

logs: artifact-dirs
	docker compose logs $(LOG_FLAGS) | tee $(ARTIFACTS_DIR)/test-logs/compose-$$(date -u +%Y%m%dT%H%M%SZ).log

shell-ros2:
	docker compose run --rm ros2 bash

shell-sitl:
	docker compose run --rm sitl bash

shell-physics:
	docker compose run --rm physics bash

test:
	docker compose --profile test run --rm test

test-e2e:
	@status=0; \
	docker compose --profile baseline --profile mujoco --profile viz down --remove-orphans || status=$$?; \
	if [ $$status -eq 0 ]; then docker compose --profile mujoco up -d sitl-json physics ros2 || status=$$?; fi; \
	if [ $$status -eq 0 ]; then docker compose exec -T ros2 bash -lc 'source /opt/ros/humble/setup.bash && python3 scripts/e2e_takeoff.py' || status=$$?; fi; \
	docker compose --profile baseline --profile mujoco --profile viz down || true; \
	exit $$status
