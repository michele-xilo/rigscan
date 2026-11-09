# Contributing to Rigscan

Thanks for helping. Rigscan is pre-alpha, so the best contributions right now
are small, tested changes that match the roadmap, and bug reports with a file
that reproduces the problem.

By contributing you agree that your work is licensed under the project's
[Apache-2.0 license](LICENSE).

## Where to start

- Open issues are the roadmap. Each has a priority (`P0`, `P1`, `P2`), a size
  (`size:S`, `size:M`, `size:L`) and a phase label. Dependencies are listed in
  the issue body.
- Comment on an issue before starting a large one, so work is not duplicated.
- For a new idea, open an issue first and describe the problem it solves.

## Reporting a bug

Include:

1. The command you ran, or the code you called.
2. The smallest robot description that shows the problem. Trim it down; a
   20-line file beats a 20,000-line one.
3. What you expected, and what happened instead.
4. Your OS and `rustc --version`.

A robot description that Rigscan flags wrongly (a false positive) or misses (a
false negative) is a bug, and a valuable one.

## Setup

You need a recent stable Rust toolchain.

```bash
git clone https://github.com/michele-xilo/rigscan
cd rigscan
rustup target add wasm32-unknown-unknown
cargo test --workspace
```

For the end-to-end tests on real robots, run `scripts/fetch-corpus.sh` once; it
downloads about 2.6 MB into the gitignored `corpus/`. Without it those tests skip.

Optional tools: `cargo-llvm-cov` for coverage and `gitleaks` for the secret
scan.

## Making a change

1. Branch from `main`. Name the branch after the work (`urdf/mimic-rule`).
2. **Write the test first** and watch it fail for the right reason. This is
   required for bug fixes and for new lint rules. Read the failure message: it
   must show the problem you are fixing, not a setup mistake.
3. Make the smallest change that turns the test green. Keep unrelated cleanup
   for a separate PR.
4. Run the checks below.
5. Update the README and docs if behaviour changed.
6. Open a pull request.

### Checks

CI runs all of these. Run them before you push:

```bash
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo build -p rigscan-core --target wasm32-unknown-unknown
cargo bench --workspace --no-run
```

### Adding a lint rule

1. Take the next free ID (`RS` plus three digits). IDs are never reused.
2. Write two tests: one that triggers the rule and asserts the exact line and
   column, and one healthy document that must not trigger it.
3. Implement the rule in `crates/rigscan-core/src/lint.rs` so that it reads only
   `Robot`, never format-specific data. One rule then serves every format.
4. Add the rule to the list in the README.
5. Add its probe to `scripts/compare/probes/` (named `RSnnn_<what>.urdf`) and a title to `TITLES` in
   `scripts/compare/compare.py`, then rerun the script (see the README comparison section).
   CI fails when a rule has no probe or the README table is stale.

### Performance

Rigscan has to be fast on large robot libraries.

- Do not optimize without a number. Show the benchmark before and after, on the
  same machine.
- Benchmarks use [criterion](https://github.com/bheisler/criterion.rs) in
  `crates/<crate>/benches/`. Name the bench target when saving a baseline:

  ```bash
  cargo bench -p rigscan-core --bench core -- --save-baseline main
  cargo bench -p rigscan-core --bench core -- --baseline main
  ```

- A slowdown of more than 5% on a hot path needs an explanation.
- Parallel work must produce the same output as serial work.

### Secrets

Never commit tokens, keys or credentials, not even fake-looking ones. CI scans
the full history with gitleaks, and GitHub secret scanning with push protection
is on. To check locally:

```bash
gitleaks detect --source . --log-opts="--all" --redact
```

## Pull requests

- One logical change per PR. Small PRs get reviewed faster.
- Describe **what** changed, **why**, and the **evidence**: which tests failed
  first, what you ran, what you could not verify.
- Use the commit style in the [style guide](STYLE_GUIDE.md#commit-messages), for
  example `urdf: ignore joints nested inside other elements`.
- `main` takes squash merges only, and a pull request needs one approving
  review.
- If CI fails, fix it in the same PR rather than opening another.

## Style

Code style, message wording and docs conventions are in the
[style guide](STYLE_GUIDE.md).

## Notes for AI coding agents

Agent-facing context, decisions and conventions live in
[docs/agent/](docs/agent/README.md), and the working rules in
[.claude/rules/](.claude/rules/). Agents follow the same workflow as everyone
else, including test-first changes.
