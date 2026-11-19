"""The Rigscan Space: scan a URDF, MJCF or USD robot and convert it with a report of what was lost."""
from pathlib import Path

import gradio as gr

import rigscan_space as space

HERE = Path(__file__).resolve().parent
EXAMPLES = sorted(str(p) for p in (HERE / "examples").glob("*") if p.is_file())
FILE_TYPES = [".urdf", ".xml", ".usd", ".usda", ".usdc"]
REPO = "https://github.com/michele-xilo/rigscan"

FINDING_HEADERS = ["rule", "severity", "line", "column", "message"]
LOSS_HEADERS = ["id", "kind", "element", "meaning"]


def do_scan(path):
    r = space.scan(path)
    if r["error"]:
        return f"**{r['error']}**", [], []
    findings = [[f["rule"], f["severity"], f["line"], f["column"], f["message"]] for f in r["findings"]]
    elements = [[e["kind"], e["name"], e["status"]] for e in r["elements"]]
    return f"**{r['verdict']}**", findings, elements


def do_convert(path, target):
    r = space.convert(path, space.TARGETS[target])
    if r["error"]:
        return f"**{r['error']}**", "", [], None, None
    rows = [[x[h] for h in LOSS_HEADERS] for x in r["losses"]]
    return f"**{r['summary']}**", r["text"], rows, r["output"], r["report_path"]


with gr.Blocks(title="Rigscan") as demo:
    gr.Markdown(
        f"# Rigscan\nA health check for robot descriptions, and a converter that says what it loses. "
        f"Version {space.version()}. [Source and docs]({REPO}), [what each loss means]({REPO}/blob/main/docs/losses.md).\n\n"
        "Your file is processed in memory for this request and is not kept. Files over 5 MB are refused. "
        "Only the uploaded file is seen, so a mesh it refers to is reported as not found."
    )
    with gr.Tab("Scan"):
        scan_file = gr.File(label="URDF, MJCF or USD file", file_types=FILE_TYPES, type="filepath")
        scan_button = gr.Button("Scan", variant="primary")
        verdict = gr.Markdown()
        findings = gr.Dataframe(headers=FINDING_HEADERS, label="Findings", interactive=False, wrap=True)
        elements = gr.Dataframe(headers=["kind", "name", "status"], label="Links and joints", interactive=False)
        scan_button.click(do_scan, scan_file, [verdict, findings, elements])
        gr.Examples(EXAMPLES, scan_file, label="Examples")
    with gr.Tab("Convert"):
        convert_file = gr.File(label="URDF, MJCF or USD file", file_types=FILE_TYPES, type="filepath")
        target = gr.Radio(list(space.TARGETS), value="MJCF", label="Convert to")
        convert_button = gr.Button("Convert", variant="primary")
        summary = gr.Markdown()
        text = gr.Code(label="Converted file", lines=16)
        losses = gr.Dataframe(headers=LOSS_HEADERS, label="What was lost", interactive=False, wrap=True)
        with gr.Row():
            converted = gr.File(label="Download the converted file")
            report = gr.File(label="Download the loss report (JSON)")
        convert_button.click(do_convert, [convert_file, target], [summary, text, losses, converted, report])
        gr.Examples(EXAMPLES, convert_file, label="Examples")

if __name__ == "__main__":
    demo.launch()
