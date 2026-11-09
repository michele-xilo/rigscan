use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rigscan_core::{lint, parse_urdf, report};
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

/// Like `chain_urdf`, but every link has an inertial and meshes and every joint
/// an origin and limits, as in a real robot.
fn rich_urdf(n: usize) -> String {
    let mut s = String::from("<robot name=\"bench\">\n");
    for i in 0..n {
        s += &format!(
            "  <link name=\"l{i}\">\n    <inertial><origin xyz=\"0 0 0.1\"/><mass value=\"1.5\"/>\
<inertia ixx=\"0.1\" ixy=\"0\" ixz=\"0\" iyy=\"0.1\" iyz=\"0\" izz=\"0.1\"/></inertial>\n\
    <visual><geometry><mesh filename=\"package://bench/meshes/l{i}.stl\"/></geometry></visual>\n\
    <collision><geometry><mesh filename=\"package://bench/meshes/l{i}_c.stl\"/></geometry></collision>\n  </link>\n"
        );
    }
    for i in 1..n {
        s += &format!(
            "  <joint name=\"j{i}\" type=\"revolute\"><parent link=\"l{}\"/><child link=\"l{i}\"/>\
<origin xyz=\"0 0 0.2\" rpy=\"0 0 0\"/><limit lower=\"-1.5\" upper=\"1.5\" effort=\"10\" velocity=\"2\"/></joint>\n",
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

fn parse_rich(c: &mut Criterion) {
    let mut g = c.benchmark_group("parse_urdf_rich");
    g.sample_size(10);
    for n in SIZES {
        let src = rich_urdf(n);
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

fn lint_rich(c: &mut Criterion) {
    let mut g = c.benchmark_group("lint_rich");
    g.sample_size(10);
    for n in SIZES {
        // A non-zero ixy sends every tensor through the eigenvalue solve, as real inertias do.
        let src = rich_urdf(n).replace("ixy=\"0\"", "ixy=\"0.01\"");
        let robot = parse_urdf(&src).unwrap();
        g.throughput(Throughput::Elements(
            (robot.links.len() + robot.joints.len()) as u64,
        ));
        g.bench_with_input(BenchmarkId::from_parameter(n), &robot, |b, robot| {
            b.iter(|| lint(black_box(robot)))
        });
    }
    g.finish();
}

fn report_all(c: &mut Criterion) {
    let mut g = c.benchmark_group("report");
    g.sample_size(10);
    for n in SIZES {
        let robot = parse_urdf(&chain_urdf(n)).unwrap();
        let findings = lint(&robot);
        g.throughput(Throughput::Elements(
            (robot.links.len() + robot.joints.len()) as u64,
        ));
        g.bench_with_input(BenchmarkId::from_parameter(n), &robot, |b, robot| {
            b.iter(|| report(black_box(robot), black_box(&findings)))
        });
    }
    g.finish();
}

criterion_group!(benches, parse, parse_rich, lint_all, lint_rich, report_all);
criterion_main!(benches);
