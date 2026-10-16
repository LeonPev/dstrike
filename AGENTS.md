Goal: run the full drone simulation stack using Docker Desktop for Mac only. No host installs are allowed except Docker Desktop, Docker Engine, and the Docker Compose plugin. Keep the first version headless and browser-viewable.

Design simulation-facing control, telemetry, and mission logic so it can later be reused with a real drone through a clearly isolated hardware adapter. Do not add real-hardware execution or testing unless explicitly requested. Never send commands to real hardware without explicit user confirmation and documented safety checks.

For every new task, always create and switch to a new Git branch before making any changes. Use a short, descriptive branch name based on the task, branch off the latest main branch, and keep all commits related to that task isolated in its dedicated branch. Never work directly on main or reuse an existing task branch.

when you finish a task write clear guidelines how can I test it manually.

whenever you see you're doing some repetitive discovery work fetching from memory how to do stuff, add a directory to the potential_skills directory with the skill name in it and a SKILL.MD file with the proposed skill.
