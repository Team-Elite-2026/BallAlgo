"""Create or update a shared Foxglove layout from a local JSON file.

This script targets the Foxglove REST API layout endpoints, which operate on
organization/shared layouts. Personal layouts are not returned by the API and
cannot be synced with this script.
"""
from __future__ import annotations

import argparse
import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import requests


API_BASE_URL = "https://api.foxglove.dev/v1"
DEFAULT_LAYOUT_NAME = "BallAlgo Live Debug"
DEFAULT_PERMISSION = "ORG_WRITE"
VALID_PERMISSIONS = {"CREATOR_WRITE", "ORG_READ", "ORG_WRITE"}

LAYOUT_DIR = Path(__file__).resolve().parent
DEFAULT_LAYOUT_PATH = LAYOUT_DIR / "ballalgo_app_layout.json"


def _utc_now_rfc3339() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _headers(api_key: str) -> dict[str, str]:
    return {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }


def _load_layout_data(path: Path) -> dict[str, Any]:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SystemExit(f"Layout file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Layout JSON is invalid: {path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise SystemExit(f"Layout JSON must be an object: {path}")
    return raw


def _request(
    method: str,
    url: str,
    *,
    api_key: str,
    params: dict[str, Any] | None = None,
    payload: dict[str, Any] | None = None,
) -> Any:
    response = requests.request(
        method,
        url,
        headers=_headers(api_key),
        params=params,
        json=payload,
        timeout=30,
    )
    try:
        response.raise_for_status()
    except requests.HTTPError as exc:
        detail = response.text[:500]
        raise SystemExit(f"Foxglove API request failed: {method} {url}: {exc}\n{detail}") from exc

    if response.content:
        return response.json()
    return None


def _list_shared_layouts(api_key: str) -> list[dict[str, Any]]:
    data = _request(
        "GET",
        f"{API_BASE_URL}/layouts",
        api_key=api_key,
        params={"includeData": "false"},
    )
    if not isinstance(data, list):
        raise SystemExit("Unexpected Foxglove API response while listing layouts.")
    return data


def _get_layout(api_key: str, layout_id: str, include_data: bool) -> dict[str, Any]:
    data = _request(
        "GET",
        f"{API_BASE_URL}/layouts/{layout_id}",
        api_key=api_key,
        params={"includeData": str(include_data).lower()},
    )
    if not isinstance(data, dict):
        raise SystemExit(f"Unexpected Foxglove API response while reading layout {layout_id}.")
    return data


def _match_layout(
    layouts: list[dict[str, Any]],
    *,
    name: str,
    folder_name: str | None,
) -> dict[str, Any] | None:
    matches = []
    for layout in layouts:
        if layout.get("name") != name:
            continue
        if folder_name is not None and layout.get("folderName") != folder_name:
            continue
        matches.append(layout)

    if not matches:
        return None
    if len(matches) > 1:
        lines = ["Multiple shared Foxglove layouts matched. Refine with --folder or --layout-id:"]
        for layout in matches:
            lines.append(
                f"  id={layout.get('id')} name={layout.get('name')} "
                f"folder={layout.get('folderName')!r} permission={layout.get('permission')}"
            )
        raise SystemExit("\n".join(lines))
    return matches[0]


def _build_payload(
    *,
    name: str,
    folder_name: str | None,
    permission: str,
    layout_data: dict[str, Any],
    layout_id: str | None = None,
    include_saved_at: bool = False,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "name": name,
        "permission": permission,
        "data": layout_data,
    }
    if include_saved_at:
        payload["savedAt"] = _utc_now_rfc3339()
    if folder_name is not None:
        payload["folderName"] = folder_name
    if layout_id is not None:
        payload["id"] = layout_id
    return payload


def _create_layout(
    api_key: str,
    *,
    name: str,
    folder_name: str | None,
    permission: str,
    layout_data: dict[str, Any],
) -> dict[str, Any]:
    return _request(
        "POST",
        f"{API_BASE_URL}/layouts",
        api_key=api_key,
        payload=_build_payload(
            name=name,
            folder_name=folder_name,
            permission=permission,
            layout_data=layout_data,
            include_saved_at=True,
        ),
    )


def _update_layout(
    api_key: str,
    layout_id: str,
    *,
    name: str,
    folder_name: str | None,
    permission: str,
    layout_data: dict[str, Any],
) -> dict[str, Any]:
    return _request(
        "PATCH",
        f"{API_BASE_URL}/layouts/{layout_id}",
        api_key=api_key,
        payload=_build_payload(
            name=name,
            folder_name=folder_name,
            permission=permission,
            layout_data=layout_data,
        ),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--file",
        type=Path,
        default=DEFAULT_LAYOUT_PATH,
        help=f"Path to the layout JSON file. Default: {DEFAULT_LAYOUT_PATH}",
    )
    parser.add_argument(
        "--name",
        default=os.environ.get("FOXGLOVE_LAYOUT_NAME", DEFAULT_LAYOUT_NAME),
        help=f"Shared layout name. Default: {DEFAULT_LAYOUT_NAME}",
    )
    parser.add_argument(
        "--folder",
        default=os.environ.get("FOXGLOVE_LAYOUT_FOLDER"),
        help="Optional shared layout folder name.",
    )
    parser.add_argument(
        "--layout-id",
        default=os.environ.get("FOXGLOVE_LAYOUT_ID"),
        help="Explicit Foxglove layout id to update. Overrides name/folder matching.",
    )
    parser.add_argument(
        "--permission",
        default=os.environ.get("FOXGLOVE_LAYOUT_PERMISSION", DEFAULT_PERMISSION),
        help=f"Layout permission: {', '.join(sorted(VALID_PERMISSIONS))}. Default: {DEFAULT_PERMISSION}",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would happen without making API changes.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    permission = args.permission.upper()
    if permission not in VALID_PERMISSIONS:
        raise SystemExit(f"Invalid --permission {args.permission!r}. Must be one of: {', '.join(sorted(VALID_PERMISSIONS))}")

    api_key = os.environ.get("FOXGLOVE_API_KEY", "")
    if not api_key:
        raise SystemExit("FOXGLOVE_API_KEY is required.")

    folder_name = args.folder
    if folder_name == "":
        folder_name = None
    if folder_name is not None and "/" in folder_name:
        raise SystemExit("Folder names cannot contain forward slashes.")

    layout_data = _load_layout_data(args.file)

    if args.layout_id:
        existing = _get_layout(api_key, args.layout_id, include_data=False)
    else:
        existing = _match_layout(
            _list_shared_layouts(api_key),
            name=args.name,
            folder_name=folder_name,
        )

    if args.dry_run:
        if existing:
            print(f"Would update shared Foxglove layout {existing['id']} ({existing['name']}).")
        else:
            print(f"Would create shared Foxglove layout {args.name!r}.")
        return 0

    if existing:
        result = _update_layout(
            api_key,
            existing["id"],
            name=args.name,
            folder_name=folder_name,
            permission=permission,
            layout_data=layout_data,
        )
        print(
            f"Updated shared Foxglove layout: id={result['id']} "
            f"name={result['name']!r} folder={result.get('folderName')!r}"
        )
    else:
        result = _create_layout(
            api_key,
            name=args.name,
            folder_name=folder_name,
            permission=permission,
            layout_data=layout_data,
        )
        print(
            f"Created shared Foxglove layout: id={result['id']} "
            f"name={result['name']!r} folder={result.get('folderName')!r}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
