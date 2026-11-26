"""Generate Taku's named instances from the standalone DynaClaw model."""

import argparse
import copy
import os
import xml.etree.ElementTree as ET
from pathlib import Path


def fragment(elements: list[ET.Element]) -> str:
    """Serialize an MJCF include fragment with a source-of-truth notice."""
    root = ET.Element("mujocoinclude")
    root.append(
        ET.Comment(
            " Generated from dyna_claw/models/dynaclaw.xml; "
            "run dyna_claw/generate_taku_includes.py after edits. "
        )
    )
    root.extend(elements)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def instance(element: ET.Element, side: str) -> ET.Element:
    """Prefix instance names and joint references, keeping shared assets intact."""
    result = copy.deepcopy(element)
    for descendant in result.iter():
        for attribute in ("name", "joint", "joint1", "joint2"):
            if attribute in descendant.attrib:
                descendant.set(attribute, f"{side}_{descendant.attrib[attribute]}")
    return result


def generate(repository: Path) -> dict[str, str]:
    """Build include contents without changing the repository."""
    source = repository / "dyna_claw/models/dynaclaw.xml"
    model = ET.parse(source).getroot()
    compiler = model.find("compiler")
    defaults = model.find("default")
    assets = model.find("asset")
    mount = model.find("worldbody/body")
    actuator = model.find("actuator")
    equality = model.find("equality")
    assert compiler is not None and defaults is not None and assets is not None
    assert mount is not None and actuator is not None and equality is not None
    source_assets = (source.parent / compiler.attrib["meshdir"]).resolve()
    asset_elements = copy.deepcopy(list(assets))
    for element in asset_elements:
        if "file" in element.attrib:
            element.set(
                "file",
                Path(
                    os.path.relpath(
                        source_assets / element.attrib["file"],
                        repository / "dyna_taku/assets",
                    )
                ).as_posix(),
            )
    result = {
        "defaults.xml": fragment(copy.deepcopy(list(defaults))),
        "assets.xml": fragment(asset_elements),
    }
    for side in ("left", "right"):
        result[f"{side}_bodies.xml"] = fragment(
            [instance(body, side) for body in mount.findall("body")]
        )
        result[f"{side}_actuator.xml"] = fragment(
            [instance(element, side) for element in actuator]
        )
    result["equalities.xml"] = fragment(
        [instance(element, side) for side in ("left", "right") for element in equality]
    )
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Check without writing.")
    arguments = parser.parse_args()
    repository = Path(__file__).resolve().parent.parent
    destination = repository / "dyna_taku/gripper"
    stale: list[str] = []
    for filename, contents in generate(repository).items():
        path = destination / filename
        if arguments.check:
            if not path.exists() or path.read_text() != contents:
                stale.append(str(path.relative_to(repository)))
        else:
            destination.mkdir(parents=True, exist_ok=True)
            path.write_text(contents)
    if stale:
        parser.exit(1, "Stale gripper includes:\n" + "\n".join(stale) + "\n")


if __name__ == "__main__":
    main()
