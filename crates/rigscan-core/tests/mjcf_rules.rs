use rigscan_core::{
    lint_mjcf, parse_mjcf, Element, Finding, IncludeLoader, Included, Resolution, Resolver, Span,
};

struct Files(Vec<(&'static str, &'static str)>);

impl IncludeLoader for Files {
    fn load(&self, file: &str, _from: Option<&str>) -> Option<Included> {
        self.0
            .iter()
            .find(|(n, _)| *n == file)
            .map(|(n, t)| Included {
                path: n.to_string(),
                text: t.to_string(),
            })
    }
}

struct Everything;

impl Resolver for Everything {
    fn resolve(&self, _uri: &str) -> Resolution {
        Resolution::Found
    }
}

fn scan_with(src: &str, files: Vec<(&'static str, &'static str)>) -> Vec<Finding> {
    let mjcf = parse_mjcf(src, &Files(files)).unwrap();
    let robot = mjcf.to_robot().unwrap();
    lint_mjcf(&mjcf, &robot, &Everything)
}

fn scan(src: &str) -> Vec<Finding> {
    scan_with(src, Vec::new())
}

fn only(findings: &[Finding], rule: &str) -> Vec<(u32, u32)> {
    findings
        .iter()
        .filter(|f| f.rule == rule)
        .map(|f| (f.span.line, f.span.col))
        .collect()
}

#[test]
fn healthy_file_has_no_findings() {
    let f = scan(
        "<mujoco>
  <default><default class=\"arm\"><geom size=\"1\"/></default></default>
  <asset><mesh name=\"m\" file=\"m.stl\"/></asset>
  <worldbody>
    <body childclass=\"arm\"><geom type=\"mesh\" mesh=\"m\"/></body>
  </worldbody>
</mujoco>",
    );
    assert!(f.is_empty(), "{f:?}");
}

#[test]
fn shared_rules_run_on_the_mapped_robot() {
    let f = scan(
        "<mujoco><worldbody>
  <body name=\"a\"><joint name=\"j\" type=\"hinge\" range=\"2 1\"/></body>
</worldbody></mujoco>",
    );
    assert_eq!(only(&f, "RS012"), [(2, 18)]);
}

#[test]
fn unknown_class_is_reported_at_the_element() {
    let f = scan(
        "<mujoco>
  <worldbody>
    <body childclass=\"nope\"><geom size=\"1\"/></body>
  </worldbody>
</mujoco>",
    );
    assert_eq!(only(&f, "RS016"), [(3, 5)]);
    let rs016 = f.iter().find(|f| f.rule == "RS016").unwrap();
    assert_eq!(rs016.message, "default class `nope` is not defined");
    assert_eq!(rs016.element, Element::Robot);
}

#[test]
fn mesh_geom_naming_an_undefined_asset_is_reported() {
    let f = scan(
        "<mujoco>
  <asset><mesh name=\"known\" file=\"k.stl\"/></asset>
  <worldbody>
    <body><geom type=\"mesh\" mesh=\"ghost\"/></body>
  </worldbody>
</mujoco>",
    );
    assert_eq!(only(&f, "RS017"), [(4, 11)]);
    let rs017 = f.iter().find(|f| f.rule == "RS017").unwrap();
    assert_eq!(rs017.message, "mesh asset `ghost` is not defined");
}

#[test]
fn asset_named_by_file_stem_or_without_file_is_defined() {
    let f = scan(
        "<mujoco>
  <asset>
    <mesh file=\"parts/link0.stl\"/>
    <mesh name=\"inline\" vertex=\"0 0 0 1 0 0 0 1 0 0 0 1\"/>
  </asset>
  <worldbody>
    <body><geom type=\"mesh\" mesh=\"link0\"/><geom type=\"mesh\" mesh=\"inline\"/></body>
  </worldbody>
</mujoco>",
    );
    assert!(only(&f, "RS017").is_empty(), "{f:?}");
}

#[test]
fn mesh_named_by_a_default_class_is_checked_too() {
    let f = scan(
        "<mujoco>
  <default><default class=\"c\"><geom type=\"mesh\" mesh=\"ghost\"/></default></default>
  <worldbody>
    <body><geom class=\"c\"/></body>
  </worldbody>
</mujoco>",
    );
    assert_eq!(only(&f, "RS017"), [(4, 11)]);
}

#[test]
fn geoms_inside_default_are_not_checked() {
    let f = scan(
        "<mujoco>
  <default><geom type=\"mesh\" mesh=\"ghost\"/></default>
  <worldbody/>
</mujoco>",
    );
    assert!(only(&f, "RS017").is_empty(), "{f:?}");
}

#[test]
fn missing_include_is_reported_at_the_include() {
    let f = scan(
        "<mujoco>
  <include file=\"gone.xml\"/>
  <worldbody/>
</mujoco>",
    );
    assert_eq!(only(&f, "RS018"), [(2, 3)]);
    let rs018 = f.iter().find(|f| f.rule == "RS018").unwrap();
    assert_eq!(rs018.message, "include file `gone.xml` was not found");
}

#[test]
fn findings_from_an_included_file_point_at_the_include() {
    let f = scan_with(
        "<mujoco>
  <include file=\"parts.xml\"/>
  <worldbody/>
</mujoco>",
        vec![(
            "parts.xml",
            "<mujoco><worldbody><body childclass=\"nope\"/></worldbody></mujoco>",
        )],
    );
    let spans: Vec<Span> = f
        .iter()
        .filter(|f| f.rule == "RS016")
        .map(|f| f.span)
        .collect();
    assert_eq!(spans.len(), 1);
    assert_eq!((spans[0].line, spans[0].col), (2, 3));
}

#[test]
fn rule_findings_are_critical() {
    let f = scan(
        "<mujoco><include file=\"x.xml\"/><worldbody><body childclass=\"c\"><geom type=\"mesh\" mesh=\"g\"/></body></worldbody></mujoco>",
    );
    for rule in ["RS016", "RS017", "RS018"] {
        let found = f
            .iter()
            .find(|f| f.rule == rule)
            .unwrap_or_else(|| panic!("{rule}"));
        assert_eq!(found.severity, rigscan_core::Severity::Critical);
    }
}
