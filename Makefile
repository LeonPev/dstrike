.PHONY: help preflight

help:
	@printf '%s\n' \
		'Targets:' \
		'  preflight  Check Docker Desktop for Mac host availability'

preflight:
	@sh scripts/preflight.sh
