# TDD for bug fixes

When fixing a bug, **write a failing test before the fix**. The sequence is non-negotiable: diagnose → failing test → minimal fix → green test → quality checks. Never write the fix and the test in the same step — if you do, you have no evidence the test actually exercises the bug.

See `modus-operandi.md` for test structure conventions (file placement, style, corpus gating). This file is about the *workflow* for bug fixes specifically. New features and new lint rules follow the same red-then-green order.

## The loop

### 1. Diagnose first

Trace the bug to its root cause before touching any code. Identify the exact file, line, and conditions that trigger it. If you can't articulate the failure precisely ("when X happens, Y returns Z instead of W"), you're not ready to write a test.

For bugs that span the pipeline (a file the parser rejects or misreads, a rule that fires wrongly, a converter that drops a feature without reporting it, a result that differs between CLI, Python and WASM), pin down which stage actually fails: the parser, the neutral model mapping, the rule, the converter, or the output layer. The bug is almost always in one place; tracking through all of them wastes time.

### 2. Write the failing test

The test must:

- **Assert the desired post-fix behaviour**, not the current buggy behaviour. The point is to encode the contract the fix must satisfy.
- **Target the smallest testable unit** that contains the bug. In Rigscan, prefer this order: a parser or rule on an in-memory string > a conversion on a small fixture > a corpus run on real robots > a manual repro. Drop one level only if the level above doesn't actually contain the bug.
- **Reproduce the bug deterministically** — no flakiness. Anything that needs the network or a large external asset cannot be the failing test; reduce the failing file to a minimal inline string and test that.

If the buggy code is hard to test (it reads files, or sits inside a long function), **extract a pure function first** — preserving the buggy behaviour exactly — and write the test against the extracted function. Refactor in a behaviour-preserving step, then test, then fix.

### 3. Run the test and verify it fails *for the right reason*

Read the failure message. It must reflect the bug you diagnosed in step 1, not a setup mistake (wrong span, bad fixture, wrong import, a typo in the input document). If the test fails for an unrelated reason, fix the test before moving on — a test that fails for the wrong reason will pass for the wrong reason later.

### 4. Apply the minimal fix

Change only what's needed to flip the failing assertion green. Resist scope creep — no surrounding cleanup, refactors, or "while I'm here" tweaks. Those go in a separate step (or PR).

### 5. Re-run the failing test, then the full local suite

The targeted test must pass. Then run `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`, `cargo fmt --all --check`, and the wasm32 build. For bugs that affect real files, rerun the corpus before declaring victory: passing unit tests do not prove real robots parse.

### 6. Communicate the trail

When reporting the fix, state explicitly: *what failed, why it failed, what changed, what now passes*. The test names should make this obvious without needing the diff.

## Anti-patterns

- **Writing the fix and test together.** You have no evidence the test exercises the bug. The test passing means nothing.
- **Asserting the buggy behaviour to "lock it in"**, then changing the assertion when fixing. The test gives no signal — it always passes against current code.
- **Skipping the failure-reason check.** A test that fails because of a missing setup will pass when you add the setup — even if your fix is wrong.
- **Bundling unrelated cleanup with the fix.** If the cleanup breaks something, the bisect is muddier and the review harder.
- **Testing through too many layers.** A bug in a rule or a parser shouldn't require the CLI or a corpus run. Extract and test the unit.
- **Hiding bugs behind ignored or skipped tests.** If the bug only repros on a large real file, the underlying logic still has a deterministic input/output you can exercise. Find it and test it.
