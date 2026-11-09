<!--
Thank you for opening a pull request! Please fill out the sections below to make the review smoother.
Structure adapted from the rerun-io/rerun pull request template (Apache-2.0).
-->

### Related

<!--
Link related issues and PRs in a bulleted list, for example:
* Closes #1234
* Part of #1337
-->

### Why

<!--
Explain why this change is needed: what problem it solves, or what would go wrong without it.
This context helps reviewers today and readers of the history later.

This section explains the problem that motivates the change; the `What` section explains how it solves it.
For a bug fix, state what failed and why.
-->

### What

<!--
Describe the change. Make sure the PR title uses the area prefix (`core:`, `cli:`, `urdf:`, `mjcf:`,
`usd:`, `convert:`, `docs:`, `ci:`); the squash commit takes it as its summary.

Call out decisions a reviewer should check, and anything that changes a contract:
rule IDs, JSON or SARIF output, public API, exit codes.
-->

### Evidence

<!--
What shows this works:
* Which tests were written first and failed for the right reason (required for bug fixes and new rules).
* Commands run: `cargo fmt --all --check`, `cargo clippy --workspace --all-targets -- -D warnings`,
  `cargo test --workspace`, `cargo build -p rigscan-core --target wasm32-unknown-unknown`.
* For parser, rule or performance changes: the corpus run, and benchmark numbers before and after
  (name the corpus commit, Rigscan commit, CPU and Rust version).

Say plainly what you could not verify.
-->

<!--
If you are a coding agent, include the section below.
Remove the comments and instructions around this section.
Optionally add information about the model that was used.

🤖 Generated with [Claude Code](https://claude.com/claude-code)
-->
