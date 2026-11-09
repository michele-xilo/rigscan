use rigscan_core::{parse_mjcf, IncludeLoader, Included, Mjcf, MjcfNode, Span};
use std::collections::HashMap;

struct Files(HashMap<&'static str, &'static str>);

impl IncludeLoader for Files {
    fn load(&self, file: &str, _from: Option<&str>) -> Option<Included> {
        self.0.get(file).map(|s| Included {
            path: file.to_string(),
            text: s.to_string(),
        })
    }
}

fn none() -> Files {
    Files(HashMap::new())
}

fn with(files: &[(&'static str, &'static str)]) -> Files {
    Files(files.iter().copied().collect())
}

fn parse(src: &str) -> Mjcf {
    parse_mjcf(src, &none()).unwrap()
}

fn find<'a>(n: &'a MjcfNode, tag: &str) -> Vec<&'a MjcfNode> {
    let mut out = Vec::new();
    if n.tag == tag {
        out.push(n);
    }
    for c in &n.children {
        out.extend(find(c, tag));
    }
    out
}

fn at(line: u32, col: u32) -> Span {
    Span { line, col }
}

#[test]
fn root_and_elements_carry_line_and_column() {
    let m = parse(
        "<mujoco model=\"r\">\n  <worldbody>\n    <body name=\"a\"/>\n  </worldbody>\n</mujoco>",
    );
    assert_eq!(m.root.tag, "mujoco");
    assert_eq!(m.root.attr("model"), Some("r"));
    assert_eq!(m.root.span, at(1, 1));
    let body = find(&m.root, "body")[0];
    assert_eq!(body.attr("name"), Some("a"));
    assert_eq!(body.span, at(3, 5));
    assert_eq!(body.file, None);
}

#[test]
fn malformed_xml_is_an_error() {
    assert!(parse_mjcf("<mujoco><worldbody></mujoco>", &none()).is_err());
}

#[test]
fn document_without_mujoco_root_is_an_error() {
    assert!(parse_mjcf("<robot/>", &none()).is_err());
}

#[test]
fn include_is_replaced_by_the_children_of_the_included_root() {
    let loader = with(&[("arm.xml", "<mujoco>\n  <body name=\"arm\"/>\n</mujoco>")]);
    let m = parse_mjcf(
        "<mujoco>\n<worldbody>\n<include file=\"arm.xml\"/>\n</worldbody>\n</mujoco>",
        &loader,
    )
    .unwrap();
    assert!(find(&m.root, "include").is_empty());
    let arm = find(&m.root, "body")[0];
    assert_eq!(arm.attr("name"), Some("arm"));
    assert_eq!(arm.span, at(2, 3));
    assert_eq!(arm.file.as_deref(), Some("arm.xml"));
    assert!(m.unresolved_includes.is_empty());
}

#[test]
fn nested_includes_are_followed() {
    let loader = with(&[
        ("a.xml", "<mujoco><include file=\"b.xml\"/></mujoco>"),
        ("b.xml", "<mujoco><body name=\"deep\"/></mujoco>"),
    ]);
    let m = parse_mjcf(
        "<mujoco><worldbody><include file=\"a.xml\"/></worldbody></mujoco>",
        &loader,
    )
    .unwrap();
    let deep = find(&m.root, "body")[0];
    assert_eq!(deep.file.as_deref(), Some("b.xml"));
}

#[test]
fn include_the_loader_cannot_supply_is_recorded_not_an_error() {
    let m = parse("<mujoco>\n  <include file=\"gone.xml\"/>\n</mujoco>");
    assert_eq!(m.unresolved_includes.len(), 1);
    assert_eq!(m.unresolved_includes[0].name, "gone.xml");
    assert_eq!(m.unresolved_includes[0].span, at(2, 3));
    assert_eq!(m.unresolved_includes[0].file, None);
    assert!(find(&m.root, "include").is_empty());
}

#[test]
fn include_cycle_is_an_error() {
    let loader = with(&[("a.xml", "<mujoco><include file=\"a.xml\"/></mujoco>")]);
    let r = parse_mjcf("<mujoco><include file=\"a.xml\"/></mujoco>", &loader);
    assert!(r.is_err());
}

const CLASSES: &str = r#"<mujoco>
  <default>
    <joint damping="1" range="-1 1"/>
    <default class="fast">
      <joint damping="5"/>
      <default class="faster">
        <geom size="0.5"/>
      </default>
    </default>
  </default>
  <worldbody>
    <body>
      <joint name="plain"/>
      <joint name="explicit" damping="9"/>
      <joint name="cls" class="fast"/>
      <geom name="nested" class="faster"/>
    </body>
    <body childclass="fast">
      <joint name="inherited"/>
      <geom name="g"/>
    </body>
  </worldbody>
</mujoco>"#;

fn joint<'a>(m: &'a Mjcf, name: &str) -> &'a MjcfNode {
    let all = find(&m.root, "joint");
    all.into_iter()
        .find(|j| j.attr("name") == Some(name))
        .unwrap()
}

#[test]
fn element_without_class_gets_the_main_default() {
    let m = parse(CLASSES);
    assert_eq!(joint(&m, "plain").attr("damping"), Some("1"));
    assert_eq!(joint(&m, "plain").attr("range"), Some("-1 1"));
}

#[test]
fn explicit_attribute_wins_over_the_default() {
    let m = parse(CLASSES);
    assert_eq!(joint(&m, "explicit").attr("damping"), Some("9"));
}

#[test]
fn class_attribute_selects_its_default_and_inherits_from_the_parent_class() {
    let m = parse(CLASSES);
    let j = joint(&m, "cls");
    assert_eq!(j.attr("damping"), Some("5"));
    assert_eq!(j.attr("range"), Some("-1 1"));
}

#[test]
fn childclass_applies_to_descendants() {
    let m = parse(CLASSES);
    assert_eq!(joint(&m, "inherited").attr("damping"), Some("5"));
}

#[test]
fn nested_class_inherits_through_two_levels() {
    let m = parse(CLASSES);
    let g = find(&m.root, "geom")
        .into_iter()
        .find(|g| g.attr("name") == Some("nested"))
        .unwrap();
    assert_eq!(g.attr("size"), Some("0.5"));
}

#[test]
fn default_for_another_tag_is_not_applied() {
    let m = parse(CLASSES);
    let g = find(&m.root, "geom")
        .into_iter()
        .find(|g| g.attr("name") == Some("g"))
        .unwrap();
    assert_eq!(g.attr("damping"), None);
}

#[test]
fn undefined_class_is_recorded_with_its_span() {
    let m = parse(
        "<mujoco>\n<worldbody>\n<body>\n<geom class=\"nope\"/>\n</body>\n</worldbody>\n</mujoco>",
    );
    assert_eq!(m.unknown_classes.len(), 1);
    assert_eq!(m.unknown_classes[0].name, "nope");
    assert_eq!(m.unknown_classes[0].span, at(4, 1));
}

#[test]
fn defined_class_is_not_reported() {
    assert!(parse(CLASSES).unknown_classes.is_empty());
}

#[test]
fn defaults_in_an_included_file_apply() {
    let loader = with(&[(
        "d.xml",
        "<mujoco><default><geom size=\"2\"/></default></mujoco>",
    )]);
    let m = parse_mjcf(
        "<mujoco><include file=\"d.xml\"/><worldbody><geom name=\"g\"/></worldbody></mujoco>",
        &loader,
    )
    .unwrap();
    assert_eq!(
        find(&m.root, "geom").last().unwrap().attr("size"),
        Some("2")
    );
}

fn nested_bodies(depth: usize) -> String {
    let mut s = String::from("<mujoco><worldbody>");
    s += &"<body>".repeat(depth);
    s += &"</body>".repeat(depth);
    s + "</worldbody></mujoco>"
}

#[test]
fn absurdly_deep_nesting_is_an_error_not_a_stack_overflow() {
    assert!(parse_mjcf(&nested_bodies(200_000), &none()).is_err());
}

#[test]
fn nesting_just_under_the_limit_still_parses() {
    assert!(parse_mjcf(&nested_bodies(250), &none()).is_ok());
}

#[test]
fn include_without_a_file_attribute_is_an_error_with_its_position() {
    let e = parse_mjcf("<mujoco>\n  <include/>\n</mujoco>", &none()).unwrap_err();
    assert!(e.0.starts_with("2:3:"), "{}", e.0);
}

#[test]
fn included_file_without_a_mujoco_root_is_an_error_naming_the_file() {
    let loader = with(&[("bad.xml", "<robot/>")]);
    let e = parse_mjcf("<mujoco><include file=\"bad.xml\"/></mujoco>", &loader).unwrap_err();
    assert!(e.0.starts_with("bad.xml:"), "{}", e.0);
}

#[test]
fn malformed_included_file_is_an_error_naming_the_file() {
    let loader = with(&[("bad.xml", "<mujoco><body></mujoco>")]);
    let e = parse_mjcf("<mujoco><include file=\"bad.xml\"/></mujoco>", &loader).unwrap_err();
    assert!(e.0.starts_with("bad.xml:"), "{}", e.0);
}

#[test]
fn two_root_elements_are_an_error() {
    assert!(parse_mjcf("<mujoco/><mujoco/>", &none()).is_err());
}

#[test]
fn unclosed_element_is_an_error() {
    assert!(parse_mjcf("<mujoco><worldbody>", &none()).is_err());
}

#[test]
fn undefined_childclass_is_recorded() {
    let m = parse("<mujoco><worldbody><body childclass=\"nope\"/></worldbody></mujoco>");
    assert_eq!(m.unknown_classes.len(), 1);
    assert_eq!(m.unknown_classes[0].name, "nope");
}

/// Resolves an include against the directory of the file that contains it, like MuJoCo.
struct Relative(HashMap<&'static str, &'static str>);

impl IncludeLoader for Relative {
    fn load(&self, file: &str, from: Option<&str>) -> Option<Included> {
        let dir = from.and_then(|f| f.rsplit_once('/')).map_or("", |(d, _)| d);
        let path = if dir.is_empty() {
            file.to_string()
        } else {
            format!("{dir}/{file}")
        };
        self.0.get(path.as_str()).map(|s| Included {
            path,
            text: s.to_string(),
        })
    }
}

fn relative(files: &[(&'static str, &'static str)]) -> Relative {
    Relative(files.iter().copied().collect())
}

#[test]
fn nested_include_is_read_relative_to_the_including_file() {
    let loader = relative(&[
        ("sub/a.xml", "<mujoco><include file=\"b.xml\"/></mujoco>"),
        ("sub/b.xml", "<mujoco><body name=\"deep\"/></mujoco>"),
    ]);
    let m = parse_mjcf(
        "<mujoco><worldbody><include file=\"sub/a.xml\"/></worldbody></mujoco>",
        &loader,
    )
    .unwrap();
    assert!(m.unresolved_includes.is_empty());
    let deep = find(&m.root, "body")[0];
    assert_eq!(deep.file.as_deref(), Some("sub/b.xml"));
}

#[test]
fn main_file_elements_are_anchored_at_their_own_position() {
    let m = parse("<mujoco>\n<worldbody>\n<body name=\"a\"/>\n</worldbody>\n</mujoco>");
    let body = find(&m.root, "body")[0];
    assert_eq!(body.anchor, body.span);
    assert_eq!(body.anchor, at(3, 1));
}

#[test]
fn included_elements_are_anchored_at_the_outermost_include() {
    let loader = with(&[
        ("a.xml", "<mujoco>\n\n<include file=\"b.xml\"/>\n</mujoco>"),
        ("b.xml", "<mujoco>\n\n\n<body name=\"deep\"/>\n</mujoco>"),
    ]);
    let m = parse_mjcf(
        "<mujoco>\n<worldbody>\n  <include file=\"a.xml\"/>\n</worldbody>\n</mujoco>",
        &loader,
    )
    .unwrap();
    let deep = find(&m.root, "body")[0];
    assert_eq!(deep.span, at(4, 1));
    assert_eq!(deep.anchor, at(3, 3));
}

#[test]
fn problems_inside_an_included_file_are_anchored_at_the_include() {
    let loader = with(&[(
        "a.xml",
        "<mujoco>\n<include file=\"gone.xml\"/>\n<worldbody><geom class=\"nope\"/></worldbody>\n</mujoco>",
    )]);
    let m = parse_mjcf("<mujoco>\n\n<include file=\"a.xml\"/>\n</mujoco>", &loader).unwrap();
    assert_eq!(m.unresolved_includes[0].span, at(2, 1));
    assert_eq!(m.unresolved_includes[0].anchor, at(3, 1));
    assert_eq!(m.unknown_classes[0].anchor, at(3, 1));
}
