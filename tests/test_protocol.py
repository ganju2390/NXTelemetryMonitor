"""Frozen wire vectors cross-checked by the independently generated C codec."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from groundstation.wire import Codec

ROOT = Path(__file__).resolve().parents[1]
VECTORS = ROOT / "protocol/test_vectors.json"


class ProtocolTests(unittest.TestCase):
    def test_python_golden_vectors_and_units(self):
        vectors = json.loads(VECTORS.read_text())
        self.assertEqual({0, 111, *range(60000, 60008)}, {v["id"] for v in vectors})
        for vector in vectors:
            codec = Codec()
            self.assertEqual(
                vector["hex"], codec.encode(vector["name"], **vector["fields"]).hex()
            )
            m = codec.decode(bytes.fromhex(vector["hex"]))[0]
            self.assertEqual(vector["id"], m.get_msgId())
            for key, value in vector["fields"].items():
                actual = getattr(m, key)
                self.assertEqual(
                    value, list(actual) if isinstance(value, list) else actual
                )
        root = ET.parse(ROOT / "protocol/finsrov.xml").getroot()
        odom = root.find("./messages/message[@name='FINS_ODOMETRY']")
        units = {f.attrib["name"]: f.get("units") for f in odom.findall("field")}
        self.assertEqual(
            ["m", "m/s", "rad", "rad/s", "us"],
            [units[n] for n in ("x", "vx", "yaw", "yaw_rate", "source_us")],
        )

    def test_c_codec_against_same_vectors(self):
        cc = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
        if not cc:
            self.skipTest("Set CC to a C11 compiler to run Python/C parity")
        vectors = json.loads(VECTORS.read_text())
        lines = [
            "#include <stdio.h>",
            "#include <string.h>",
            '#include "finsrov/mavlink.h"',
            "int main(void) { uint8_t wire[300]; mavlink_message_t m, parsed; mavlink_status_t status; int accepted;",
        ]
        for i, v in enumerate(vectors):
            name = v["name"].lower()
            init = []
            for key, value in v["fields"].items():
                literal = (
                    "{" + ",".join(map(str, value)) + "}"
                    if isinstance(value, list)
                    else str(value)
                )
                init.append("." + key + "=" + literal)
            data = bytes.fromhex(v["hex"])
            lines.extend(
                [
                    "{",
                    f"mavlink_{name}_t input = {{" + ",".join(init) + "};",
                    f"mavlink_{name}_t output;",
                    "uint8_t expected[] = {" + ",".join(map(str, data)) + "};",
                    "memset(mavlink_get_channel_status(MAVLINK_COMM_0),0,sizeof(mavlink_status_t));",
                    f"mavlink_msg_{name}_encode(255,190,&m,&input);",
                    f"if(m.msgid != {v['id']}) return {10 + i};",
                    "unsigned n=mavlink_msg_to_send_buffer(wire,&m);",
                    f"if(n!=sizeof(expected)||memcmp(wire,expected,n)) return {30 + i};",
                    "accepted=0; for(unsigned j=0;j<n;j++) accepted+=mavlink_parse_char(MAVLINK_COMM_1,expected[j],&parsed,&status);",
                    f"if(accepted!=1) return {50 + i};",
                    f"mavlink_msg_{name}_decode(&parsed,&output);",
                ]
            )
            for key, value in v["fields"].items():
                if isinstance(value, list):
                    lines.append(
                        f"for(unsigned k=0;k<{len(value)};k++) if(output.{key}[k]!=input.{key}[k]) return {70 + i};"
                    )
                else:
                    lines.append(f"if(output.{key}!=input.{key}) return {70 + i};")
            lines.extend(
                [
                    "expected[n-1]^=1;",
                    "accepted=0; for(unsigned j=0;j<n;j++) accepted+=mavlink_parse_char(MAVLINK_COMM_1,expected[j],&parsed,&status);",
                    f"if(accepted!=0) return {90 + i};",
                    "}",
                ]
            )
        lines.append(
            'puts("10 Python/C vectors and CRC corruption checks passed"); return 0; }'
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            source = path / "probe.c"
            binary = path / ("probe.exe" if os.name == "nt" else "probe")
            source.write_text("\n".join(lines))
            env = dict(
                os.environ,
                PATH=str(Path(cc).parent) + os.pathsep + os.environ.get("PATH", ""),
            )
            compiled = subprocess.run(
                [
                    cc,
                    "-std=c11",
                    "-Wno-address-of-packed-member",
                    "-I",
                    str(ROOT / "protocol/generated/c"),
                    str(source),
                    "-o",
                    str(binary),
                ],
                capture_output=True,
                text=True,
                timeout=60,
                env=env,
            )
            self.assertEqual(0, compiled.returncode, compiled.stderr)
            tested = subprocess.run(
                [str(binary)], capture_output=True, text=True, timeout=10
            )
            self.assertEqual(0, tested.returncode, tested.stdout + tested.stderr)


if __name__ == "__main__":
    unittest.main()
