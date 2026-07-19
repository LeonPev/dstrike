# dsriker map ticket flow

Use for numbered MAP tickets in this repository.

1. Read `AGENTS.md` and the exact ticket in `mountain_terrain_map_plan.md`.
2. Confirm the worktree is clean and the previous MAP ticket is committed.
3. Create the plan-specified `feature/MAP-NNN-*` branch from that tip.
4. Keep all runtime and verification dependencies inside Docker Compose.
5. Run the ticket's exact automated check and capture its required PASS line.
6. Commit the completed ticket, then report manual browser/CLI signoff and cleanup commands.
