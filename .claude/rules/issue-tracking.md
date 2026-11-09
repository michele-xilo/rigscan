# Issue tracking

GitHub issues in `michele-xilo/rigscan` are the project board. Nothing that needs work later lives only in
chat, a code comment or a PR description.

## When to file

- **A feature request from the user:** always file an issue, then ask whether to start it now.
- **A bug you detect** (in the code, a rule, a doc, or a result that looks wrong): triage it, as below.
- Search open issues first (`gh issue list --search "<keywords>"`). Comment on an existing one rather than
  filing a duplicate.

## Triage a bug: small or big

A bug is **small** only if all of these hold:

- the root cause is understood and sits in one place;
- the fix is a few lines, with no design decision;
- it changes no contract: rule IDs, JSON or SARIF output, public API, exit codes;
- it does not need a benchmark or corpus run to judge.

**Small:** fix it now, following `TDD.md` (failing test, right failure reason, minimal fix, green). Keep it in
its own commit, or its own branch if the current branch is for other work, so the change and its bisect
stay clean. Say what failed in the PR description. No issue is needed unless the user asks for one.

**Big (anything else):** do not fix it in passing. File an issue with a priority and carry on with the
current task. Tell the user in one line what was filed. Start the fix only when the user says so.

When unsure which it is, treat it as big.

## What an issue contains

- Title: short, imperative or descriptive, no prefix.
- Body: what is wrong or wanted, how to reproduce (smallest input, command, expected vs actual), the
  affected area (parser, rule, converter, CLI, bindings), and `**Depends on:** #n` when relevant.
- Labels: one priority, one size, one phase. Reuse the existing labels; do not invent new ones.
- No credentials, tokens or large third-party robot assets in the body.

## Priority

Assign it yourself, and say why in a line. The user can change it.

- **P0:** wrong results that users would trust (a broken robot scans clean, a false critical on a healthy
  file, a crash or silent data loss in a conversion), or a blocker for the current phase.
- **P1:** wrong or missing behaviour with a workaround, a missing rule or format feature on the roadmap,
  or a measured performance regression over 5% on a hot path.
- **P2:** polish, docs, tooling, distribution, ideas for later.

Size: `size:S` under a day, `size:M` a few days, `size:L` more than that or needs a spike first.

## Closing the loop

- A PR that fixes an issue says `Closes #n`.
- A decision taken while working on an issue goes on that issue as a comment, and in
  `docs/agent/decisions.md` if it changes a standing decision.
- Do not close an issue because a PR is open; it closes when the work merges.
