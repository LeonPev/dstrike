.PHONY: help preflight build up up-baseline baseline-heartbeat up-mujoco probe-ros-visualization test-json-stub test-mujoco-quad test-frame-conventions test-ros-frame-conventions up-viz down logs shell-ros2 shell-sitl shell-physics test test-e2e

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
		'  test-json-stub Run JSON protocol stub unit tests in Docker' \
		'  test-mujoco-quad  Run the minimal MuJoCo quad self-test in Docker' \
		'  test-frame-conventions  Run ENU/NED and altitude sign unit tests' \
		'  test-ros-frame-conventions  Run ROS odometry frame-convention unit tests' \
		'  up-viz         Start the browser visualization placeholder on localhost:8765' \
		'  down           Stop and remove Compose services' \
		'  logs           Follow Compose service logs' \
		'  shell-ros2     Open a shell in the ros2 service image' \
		'  shell-sitl     Open a shell in the sitl service image' \
		'  shell-physics  Open a shell in the physics service image' \
		'  test           Run scaffold smoke checks in Docker' \
		'  test-e2e       Run scaffold e2e placeholder checks in Docker'

preflight:
	@sh scripts/preflight.sh

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

test-json-stub:
	docker compose run --rm physics python3 -m pytest tests/test_ardupilot_json_stub.py

test-mujoco-quad:
	docker compose run --rm physics python3 -m src.physics.self_test

test-frame-conventions:
	docker compose run --rm physics pytest tests/test_frame_conventions.py

test-ros-frame-conventions:
	docker compose run --rm ros2 pytest tests/test_ros_frame_conventions.py

up-viz:
	docker compose --profile viz up -d viz

down:
	docker compose down

logs:
	docker compose logs -f

shell-ros2:
	docker compose run --rm ros2 bash

shell-sitl:
	docker compose run --rm sitl bash

shell-physics:
	docker compose run --rm physics bash

test:
	docker compose --profile test run --rm test

test-e2e:
	docker compose --profile test run --rm test bash -lc 'test -f /workspace/compose.yaml && test -d /workspace/models/scenarios && printf "scaffold-e2e-ok\n"'
