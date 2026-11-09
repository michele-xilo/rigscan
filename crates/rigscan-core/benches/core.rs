use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rigscan_core::{lint, parse_urdf};
use std::hint::black_box;

/// A chain of `n` links joined by `n - 1` fixed joints, one element per line.
fn chain_urdf(n: usize) -> String {
    let mut s = String::from("<robot name=\"bench\">\n");
    for i in 0..n {
        s += &format!("  <link name=\"l{i}\"/>\n");
    }
    for i in 1..n {
        s += &format!(
            "  <joint name=\"j{i}\" type=\"fixed\"><parent link=\"l{}\"/><child link=\"l{i}\"/></joint>\n",
            i - 1
        );
    }
    s + "</robot>"
}

// Few samples: before the quadratic fixes the largest size takes seconds per iteration.
const SIZES: [usize; 3] = [1_000, 4_000, 16_000];

fn parse(c: &mut Criterion) {
    let mut g = c.benchmark_group("parse_urdf");
    g.sample_size(10);
    for n in SIZES {
        let src = chain_urdf(n);
        g.throughput(Throughput::Bytes(src.len() as u64));
        g.bench_with_input(BenchmarkId::from_parameter(n), &src, |b, src| {
            b.iter(|| parse_urdf(black_box(src)).unwrap())
        });
    }
    g.finish();
}

fn lint_all(c: &mut Criterion) {
    let mut g = c.benchmark_group("lint");
    g.sample_size(10);
    for n in SIZES {
        let robot = parse_urdf(&chain_urdf(n)).unwrap();
        g.throughput(Throughput::Elements(
            (robot.links.len() + robot.joints.len()) as u64,
        ));
        g.bench_with_input(BenchmarkId::from_parameter(n), &robot, |b, robot| {
            b.iter(|| lint(black_box(robot)))
        });
    }
    g.finish();
}

criterion_group!(benches, parse, lint_all);
criterion_main!(benches);
