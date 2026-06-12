from __future__ import annotations

import argparse
import json
import subprocess
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from artifact_io import read_artifact


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUILD_DIR = REPO_ROOT / "build"
DEFAULT_GENERATED_DIR = REPO_ROOT / "tests" / "trajectory_cases" / "generated"
EDITOR_HTML = Path(__file__).with_name("case_editor.html")
PLANNER_BINARY = "ballalgo_planner_case"


def load_html() -> bytes:
    return EDITOR_HTML.read_bytes()


def run_preview(case: dict, build_dir: Path, generated_dir: Path) -> dict:
    generated_dir.mkdir(parents=True, exist_ok=True)
    output_path = generated_dir / "editor_preview.traj"
    binary = build_dir / PLANNER_BINARY
    if not binary.exists():
        raise FileNotFoundError(f"planner binary not found: {binary}")

    start = case["start_pose"]
    goal = case["goal_pose"]
    command = [
        str(binary),
        "--case-name",
        case.get("case_name", "editor_preview"),
        "--output",
        str(output_path),
        "--start",
        str(start["x_mm"]),
        str(start["y_mm"]),
        str(start["heading_deg"]),
        "--goal",
        str(goal["x_mm"]),
        str(goal["y_mm"]),
        str(goal["heading_deg"]),
    ]
    vx_mm_s = float(start.get("vx_mm_s", 0.0))
    vy_mm_s = float(start.get("vy_mm_s", 0.0))
    if abs(vx_mm_s) > 1e-6 or abs(vy_mm_s) > 1e-6:
        command.extend(["--start-velocity-field", str(vx_mm_s), str(vy_mm_s)])

    subprocess.run(command, check=True)
    artifact = read_artifact(output_path)
    return {
        "artifact_path": str(output_path),
        "artifact": artifact,
    }


class CaseEditorHandler(BaseHTTPRequestHandler):
    build_dir: Path
    generated_dir: Path

    def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, body: bytes) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        route = urlparse(self.path).path
        if route in {"/", "/index.html"}:
            self._send_html(load_html())
            return
        self.send_error(HTTPStatus.NOT_FOUND, "Not Found")

    def do_POST(self) -> None:  # noqa: N802
        route = urlparse(self.path).path
        if route != "/api/generate":
            self.send_error(HTTPStatus.NOT_FOUND, "Not Found")
            return

        length = int(self.headers.get("Content-Length", "0"))
        payload = self.rfile.read(length)
        try:
            case = json.loads(payload.decode("utf-8"))
            result = run_preview(case, self.build_dir, self.generated_dir)
        except subprocess.CalledProcessError as exc:
            self._send_json(
                {"error": f"planner command failed with exit code {exc.returncode}"},
                status=HTTPStatus.BAD_GATEWAY,
            )
            return
        except Exception as exc:  # noqa: BLE001
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
            return

        self._send_json(result)


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve the trajectory case editor")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host")
    parser.add_argument("--port", type=int, default=8765, help="Bind port")
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=DEFAULT_BUILD_DIR,
        help="BallAlgo build directory containing ballalgo_planner_case",
    )
    parser.add_argument(
        "--generated-dir",
        type=Path,
        default=DEFAULT_GENERATED_DIR,
        help="Where to write the editor preview artifact",
    )
    args = parser.parse_args()

    handler_cls = type(
        "BoundCaseEditorHandler",
        (CaseEditorHandler,),
        {"build_dir": args.build_dir, "generated_dir": args.generated_dir},
    )
    server = ThreadingHTTPServer((args.host, args.port), handler_cls)
    print(f"Trajectory case editor listening on http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
