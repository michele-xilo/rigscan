//! Rotations converted to the URDF `rpy` convention: fixed axes x, y, z, so R = Rz(yaw) Ry(pitch) Rx(roll).

pub(crate) type Mat = [[f64; 3]; 3];

const IDENTITY: Mat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Below this a vector has no direction.
const TINY: f64 = 1e-12;

pub(crate) fn mul(a: &Mat, b: &Mat) -> Mat {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    out
}

fn unit(v: [f64; 3]) -> Option<[f64; 3]> {
    let n = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    (n > TINY).then(|| v.map(|x| x / n))
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Rotation by `angle` radians about the unit `axis` (Rodrigues).
fn rotation(axis: [f64; 3], angle: f64) -> Mat {
    let (s, c) = angle.sin_cos();
    let [x, y, z] = axis;
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

/// Rotation about the coordinate axis `index` (0 x, 1 y, 2 z).
pub(crate) fn about(index: usize, angle: f64) -> Mat {
    let mut axis = [0.0; 3];
    axis[index] = 1.0;
    rotation(axis, angle)
}

/// `None` when the axis has no direction.
pub(crate) fn from_axis_angle(axis: [f64; 3], angle: f64) -> Option<Mat> {
    unit(axis).map(|a| rotation(a, angle))
}

/// A quaternion given as w, x, y, z; `None` when it is zero.
pub(crate) fn from_quat(q: [f64; 4]) -> Option<Mat> {
    let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    if n <= TINY {
        return None;
    }
    let [w, x, y, z] = q.map(|v| v / n);
    Some([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])
}

/// The frame whose x axis is along `x` and whose xy plane contains `y`; `None` when either has no
/// direction or they are parallel.
pub(crate) fn from_xyaxes(x: [f64; 3], y: [f64; 3]) -> Option<Mat> {
    let x = unit(x)?;
    let d = dot(x, y);
    let y = unit([y[0] - d * x[0], y[1] - d * x[1], y[2] - d * x[2]])?;
    let z = cross(x, y);
    Some([[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]])
}

/// The shortest rotation taking the z axis to `z`; `None` when `z` has no direction.
pub(crate) fn from_zaxis(z: [f64; 3]) -> Option<Mat> {
    let z = unit(z)?;
    match unit(cross([0.0, 0.0, 1.0], z)) {
        Some(axis) => Some(rotation(axis, z[2].clamp(-1.0, 1.0).acos())),
        // Parallel to z: nothing to turn, or a half turn about x.
        None if z[2] > 0.0 => Some(IDENTITY),
        None => Some(about(0, std::f64::consts::PI)),
    }
}

/// Roll, pitch, yaw of `r`. At gimbal lock (pitch of +-90 degrees) roll and yaw are not unique;
/// roll is taken as 0.
pub(crate) fn rpy(r: &Mat) -> [f64; 3] {
    let pitch = (-r[2][0]).clamp(-1.0, 1.0).asin();
    if r[2][0].abs() > 1.0 - 1e-9 {
        return [0.0, pitch, (-r[0][1]).atan2(r[1][1])];
    }
    [r[2][1].atan2(r[2][2]), pitch, r[1][0].atan2(r[0][0])]
}
