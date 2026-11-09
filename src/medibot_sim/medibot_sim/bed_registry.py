#!/usr/bin/env python3
"""
bed_registry.py — Bed location lookup service.
Loads bed IDs and their map coordinates from beds.yaml.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory


class BedRegistry:
    """Lookup table for bed locations and hospital destinations."""

    def __init__(self, config_path: str = None):
        if config_path is None:
            pkg_dir = get_package_share_directory('medibot_sim')
            config_path = os.path.join(pkg_dir, 'config', 'beds.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.beds = self.config.get('beds', {})
        self.pharmacy = self.config.get('pharmacy', {})
        self.nurse_station = self.config.get('nurse_station', {})
        self.robot_spawn = self.config.get('robot_spawn', {})

    def get_bed(self, bed_id: str) -> dict:
        """Look up a bed by ID (e.g., 'bed_01')."""
        normalized = bed_id.lower().replace(' ', '_').replace('-', '_')
        if not normalized.startswith('bed_'):
            normalized = f'bed_{normalized}'
        return self.beds.get(normalized)

    def get_pharmacy(self) -> dict:
        """Return pharmacy station coordinates."""
        return self.pharmacy

    def list_beds(self) -> list:
        """Return list of all bed IDs."""
        return list(self.beds.keys())
