"""cm (pure python): load and resolve config/assets.json — NO omni/pxr imports."""
from __future__ import annotations

import json
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class AssetConfig:
    def __init__(self, data: dict):
        self._d = data

    @property
    def asset_root(self) -> str:
        return self._d["asset_root"]

    @property
    def isaac_python(self) -> str:
        return self._d["isaac_python"]

    @property
    def ext_folder(self) -> str:
        return self._d["ext_folder"]

    def _resolve(self, rel: str) -> str:
        return f"{self.asset_root}/{rel}"

    def env_usd(self, variant: str) -> str:
        return self._resolve(self._d["environments"][variant])

    def env_variants(self) -> list[str]:
        return list(self._d["environments"].keys())

    def robot_usd(self, name: str) -> str:
        return self._resolve(self._d["robots"][name])

    def prop_usd(self, name: str) -> str:
        return self._resolve(self._d["props"][name])

    def conveyor_usd(self, module: str) -> str:
        return f"{self._resolve(self._d['props']['conveyor_dir'])}/{module}.usd"


def load_assets() -> AssetConfig:
    with open(os.path.join(REPO_ROOT, "config", "assets.json"), encoding="utf-8") as f:
        return AssetConfig(json.load(f))
