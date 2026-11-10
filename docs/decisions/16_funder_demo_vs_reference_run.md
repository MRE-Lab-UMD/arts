# Decision 16 — Funder demo vs ARTS reference run

> **Role: decision record.** Moved verbatim from `AGENTS.md` on 2026-10-26 so the agent
> onboarding file stays short; `AGENTS.md` keeps a summary and links here.
> Words like "above", "below" or "this file" in the text refer to
> `AGENTS.md` as it was laid out before the move.

## Two separate deliverables, both called "the demo" until 2026-08-19 — keep them apart

Conflating these two cost a paragraph of wrong requirements in this file, so the vocabulary is
now fixed and used consistently everywhere below.

| | **Funder demo** | **ARTS reference run** |
|---|---|---|
| When | 2026-10-05, fixed | any time before the Month 3 writing window |
| Stack | Ubuntu 22.04 + ROS 2 Humble, the EXISTING system | Ubuntu 24.04 + ROS 2 Jazzy, i.e. ARTS |
| Task | teleoperated partial nephrectomy on a hydrogel phantom, monopolar curved scissors + bipolar fenestrated forceps | the same task |
| Its job | convince the funder. **If it works, it works.** | produce the ISMR example dataset and every number in the paper |
| Benchmarked? | **NO.** Not instrumented, not measured, not a data source. | Yes — this is where all measurement happens |

**DECIDED 2026-08-19: NO DATA FROM THE FUNDER DEMO GOES INTO THE PAPER, even if it is recorded
anyway.** The earlier plan had ARTS's recorder running alongside at the demo to capture it as
the example dataset, and that is now dropped rather than deferred. The reason is not scheduling
convenience, it is that the resulting dataset would describe the wrong system: legacy-stack
timing, no ESP32 sync box, no `camera_time`/pulse-index alignment chain, and none of the
recording rules of decisions 12 and 15 — so every sync, alignment and jitter figure the paper
reports would be unsupported by the data it shipped. A reviewer asking "is this dataset from
the system you described" would be right to ask, and the honest answer would be no.
The corollary is what makes this cheap: **the funder demo has no ARTS deliverable at all.**
The sync box, the camera pair and the MCAP recorder do not have to be present or working on
Oct 5, which removes a hard external date from ARTS's hardware critical path.
Phases 0–4 stay the critical path regardless, because the PAPER needs them — the change is
that a slip no longer takes a funder commitment down with it.

**Collaborators targeting Humble are CORRECT, not behind.** The Oct 5 demo runs on Humble, so
work aimed at it (the ESU-TAP firmware is the live example) is right for its purpose. The
Humble→Jazzy migration is ARTS's job at intake time, never a change requested upstream.

Phases 5 (sim) and 6 (Docker release) may slip past the reference run — they are release and
sim concerns, not prerequisites for recording the task. Stapler and vessel-sealing tests come
after it too.

ISMR schedule: Month 1 = phases 0–2 + latency rig. Month 2 = phases 3–4, then the ARTS
reference run (automated metrics accumulate during dev). Month 3 = NASA-TLX study + remaining
tissue tests + writing + anonymized project page; submission at T+3 months (~early Nov 2026).
RESCOPED 2026-10-20: the ISMR deadline is **2026-11-10** and Phases 2–4 are not built, so the
schedule above no longer holds. The 20-day plan, the experiment cut list and the revised
paper claim live in the manuscript repo's `publication_plan.md`; anything cut there is
future work for the paper, not abandoned for ARTS. The production workstation and the
sync-box PCB both arrived 2026-10-20.
Minimal publishable set if short on time: system description + latency + recording/sync + RCM
+ the reference-run task with NASA-TLX (stapler/vessel-seal as qualitative demos or future
work).

Double-blind logistics: this private GitHub repo stays private and is never linked from the
submission (repo → author identity → desk rejection risk). For review: anonymized project
page on a fresh GitHub org/Pages (authors redacted) + anonymized code capsule (e.g.
anonymous.4open.science). De-anonymize after acceptance.
