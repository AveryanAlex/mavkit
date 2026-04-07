import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
README_PATH = REPO_ROOT / "mavkit-python" / "README.md"
RUST_EXAMPLES_DIR = REPO_ROOT / "examples"
PYTHON_EXAMPLES_DIR = REPO_ROOT / "mavkit-python" / "examples"


def _example_stems(directory: Path, suffix: str) -> set[str]:
    return {path.stem for path in directory.glob(f"*{suffix}") if path.is_file()}


def _readme_example_scripts() -> set[str]:
    scripts: set[str] = set()
    for line in README_PATH.read_text().splitlines():
        if not line.startswith("| `") or ".py`" not in line:
            continue
        match = re.match(r"\| `([^`]+\.py)` \|", line)
        assert match is not None, f"README examples row is malformed: {line}"
        scripts.add(match.group(1))
    return scripts


class TestExamples:
    def test_rust_and_python_examples_have_matching_stems(self):
        rust_stems = _example_stems(RUST_EXAMPLES_DIR, ".rs")
        python_stems = _example_stems(PYTHON_EXAMPLES_DIR, ".py")

        assert rust_stems == python_stems, (
            "Rust/Python example drift detected. "
            f"Missing Python examples: {sorted(rust_stems - python_stems)}. "
            f"Unexpected Python examples: {sorted(python_stems - rust_stems)}."
        )

    def test_readme_examples_table_matches_python_examples(self):
        python_scripts = {
            f"{stem}.py" for stem in _example_stems(PYTHON_EXAMPLES_DIR, ".py")
        }
        readme_scripts = _readme_example_scripts()

        assert readme_scripts == python_scripts, (
            "README examples table drift detected. "
            f"Missing README rows: {sorted(python_scripts - readme_scripts)}. "
            f"Unexpected README rows: {sorted(readme_scripts - python_scripts)}."
        )

    def test_readme_guided_actions_point_to_ardupilot_session_api(self):
        readme = README_PATH.read_text()

        assert "vehicle.ardupilot().guided()" in readme
        assert "vehicle.guided()" not in readme
        assert "Vehicle.guided()" not in readme

    def test_readme_mission_example_uses_current_plan_shape(self):
        readme = README_PATH.read_text()

        assert "mission_type=" not in readme
        assert "seq=" not in readme
        assert "current=" not in readme
