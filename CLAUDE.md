@AGENTS.md

## Notes for Claude Code

- `AGENTS.md` above is the shared onboarding file for every agent tool; edit it, not this
  file, for anything that is not Claude-Code-specific. Keep it short (see its first section).
- Before editing in an area, read its decision record in `docs/decisions/` and its module doc
  in `docs/modules/`.
- Commands needing `sudo` (apt, dkms, GRUB, udev) are run by the user: print the exact
  command, explain what it changes, and ask for the output.
- Run `deploy/ci/check_anonymity.sh` before every commit. The git identity is repo-local;
  this machine has no GitHub credentials, so the user pushes.
- Sibling trees: `~/arts_local` (machine-local tree 2, including `code_intake/`) and
  `~/arts-manuscript` (the paper; it has its own `CLAUDE.md`). A session started here does
  not load their guidance: before reviewing intake code read the `NOTES.md` of that
  `code_intake/<source>/`, and before writing any manuscript text read
  `~/arts-manuscript/CLAUDE.md` and `~/arts-manuscript/.claude/skills/ieee-writing/SKILL.md`.
