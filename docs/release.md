# Releasing

Publishing a GitHub release deploys two things from the same tag:

- the browser viewer (`web/`, built by `scripts/build-web.sh`) to GitHub Pages;
- the Gradio app (`space/`) to the Hugging Face Space, with the release's Linux `rigscan` binary next to `app.py`
  (`scripts/stage-space.sh`).

Both show the version they were built from.

## Cutting a release

1. Set `version` in the workspace `Cargo.toml` and merge it to `main`.
2. Publish a GitHub release whose tag is `v` plus that version (`v0.1.0`). The workflow stops, and deploys nothing, if
   the tag and the version differ.
3. The `Release` workflow runs three jobs: `version`, `pages` and `space`. Run it by hand from the Actions tab
   (`workflow_dispatch`) to redeploy a ref without a new release.

## One-time setup

- Pages: Settings, Pages, source "GitHub Actions". The `github-pages` environment must allow deployments from the
  release tags (a `v*` tag rule), not only from `main`.
- Space: create a Gradio Space on Hugging Face, then set the repository variable `HF_SPACE` (`owner/name`) and the
  repository secret `HF_TOKEN` (a token with write access to that Space). The token lives only in the secret.

## Checking the Space locally

```bash
cargo build --release -p rigscan-cli
scripts/stage-space.sh space-build
pip install -r space/requirements.txt
cd space-build && python3 app.py
```

`python3 space/test_rigscan_space.py` tests the app's logic against the built CLI without Gradio; CI runs it.
