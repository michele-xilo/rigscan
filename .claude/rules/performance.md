# Performance

Rigscan has to be fast enough that people run it on every save and on whole robot libraries in CI. Speed
is a feature, and it is measured, not assumed.

## Measure first

- Benchmarks use [criterion](https://github.com/bheisler/criterion.rs), in each crate's `benches/`, with
  `harness = false`. Add criterion as a dev-dependency when the first benchmark is written, not before.
- Every parser, rule set and converter gets a benchmark on realistic input (a real robot from the corpus,
  plus a synthetic large model with thousands of links) before it is called done. Benchmark throughput
  (bytes/s or elements/s), not only time.
- Do not optimize without a number. A change made for speed states the benchmark before and after, on the
  same machine, and is rejected if the gain is within noise.
- Compare against a baseline: `cargo bench -p rigscan-core --bench core -- --save-baseline main` on `main`,
  then the same with `--baseline main` on the branch. Name the bench target: without `--bench`, cargo
  also passes the flag to the lib's test harness, which rejects it. A regression of more than 5% on a hot path needs an
  explanation in the change.
- CI only checks that benches compile (`cargo bench --no-run`). Timings on shared runners are too noisy to
  gate on.
- Every reported speed names the corpus, the Rigscan commit, the CPU and the Rust version.

## Design for speed from the start

- Parse once and borrow. Prefer `&str` slices and `Cow` over `String` clones; own data only when it must
  outlive the source text.
- Compute spans in one pass. Do not recount lines from the start of the file for each element; keep a
  running line offset, or a line-start table with a binary search.
- Use indexed lookups for names, not repeated linear scans. Build a name-to-index map once per model and
  share it across rules.
- Preallocate (`Vec::with_capacity`) when the size is known or estimable. Keep hot loops free of
  allocation and formatting; build message strings only when a finding is actually produced.
- Each rule should be a single linear pass over the model where possible. Avoid O(n^2) over links and
  joints: robot models reach thousands of bodies.
- Pick a faster hasher (for example `rustc-hash`) only when a benchmark shows hashing matters, and add it
  then.

## Parallelism

- Scanning many files is embarrassingly parallel: the CLI uses `rayon` to scan and convert files
  concurrently.
- Inside `rigscan-core`, parallelism sits behind an optional `parallel` cargo feature that is off by
  default. The wasm32 build must work without it, because browser threads need extra setup.
- Parallelize only above a size threshold, found by benchmark. Spawning work for a small robot costs more
  than it saves.
- Output must be deterministic. Findings are sorted by file, then span, then rule ID before they are
  reported, so a parallel run and a serial run print the same bytes. A test asserts this.
- Shared state in parallel code is a smell. Prefer independent work per file or per rule, with results
  merged at the end.

## Keep it honest

- Clarity still matters. Take the simple version unless a benchmark shows it is the bottleneck, then
  optimize that spot and leave a comment explaining the measured reason.
- No `unsafe` for speed without a benchmark showing it is needed and a comment stating the invariant.
- Check performance claims in the browser too: WASM speed and binary size matter for the playground, so
  note both when a change touches the core.
