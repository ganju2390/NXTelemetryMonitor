"""Generate Python and C from the checked-in dialect using locked pymavlink."""

from pathlib import Path
import shutil
import pymavlink
from pymavlink.generator import mavgen

ROOT = Path(__file__).resolve().parents[1]


def main():
    # minimal.xml and its standard.xml include (if any) come from the locked toolchain.
    source = Path(pymavlink.__file__).parent / "dialects/v20"
    for name in ("minimal.xml",):
        shutil.copyfile(source / name, ROOT / "protocol" / name)
    for language, output in (
        ("Python3", ROOT / "groundstation/generated/finsrov"),
        ("C", ROOT / "protocol/generated/c"),
    ):
        output.parent.mkdir(parents=True, exist_ok=True)
        options = mavgen.Opts(str(output), wire_protocol="2.0", language=language)
        if not mavgen.mavgen(options, [str(ROOT / "protocol/finsrov.xml")]):
            raise RuntimeError("MAVLink generation failed")


if __name__ == "__main__":
    main()
