"""Reference values transcribed from GB/T 46851-2025 informative Annex B.

Important: Table B.1 is an example for a 100 m bulk carrier, not a USV-specific
minimum-distance rule. These values are only exposed as an optional reference;
USV users must calibrate an appropriate SDA for their vehicle size and dynamics.
All distances below are nautical miles.
"""
from __future__ import annotations
from .geometry import NM_TO_M

# B.1 example: own ship is a 100 m bulk carrier; target ship length is 50/100/200/300 m.
# A table cell is (target length m) -> SDA in nmi.
SDA_B1_100M_BULK = {
    "head_on": {50: 0.34, 100: 0.40, 200: 0.50, 300: 0.60},
    "crossing_starboard_bow": {50: 0.45, 100: 0.52, 200: 0.61, 300: 0.73},
    "crossing_starboard_stern": {50: 0.43, 100: 0.47, 200: 0.56, 300: 0.70},
    "crossing_port_bow": {50: 0.43, 100: 0.49, 200: 0.58, 300: 0.69},
    "crossing_port_stern": {50: 0.42, 100: 0.44, 200: 0.52, 300: 0.62},
    "beam_starboard_bow": {50: 0.42, 100: 0.50, 200: 0.60, 300: 0.70},
    "beam_starboard_stern": {50: 0.38, 100: 0.41, 200: 0.50, 300: 0.60},
    "beam_port_bow": {50: 0.44, 100: 0.50, 200: 0.59, 300: 0.75},
    "beam_port_stern": {50: 0.38, 100: 0.42, 200: 0.52, 300: 0.63},
    "being_overtaken_pass_left_or_right": {50: 0.33, 100: 0.40, 200: 0.50, 300: 0.60},
    "overtaking_pass_left_or_right": {50: 0.36, 100: 0.40, 200: 0.50, 300: 0.60},
    "right_rear_overtake_bow": {50: 0.41, 100: 0.50, 200: 0.60, 300: 0.70},
    "right_rear_overtake_stern": {50: 0.38, 100: 0.41, 200: 0.50, 300: 0.60},
    "left_rear_overtake_bow": {50: 0.41, 100: 0.50, 200: 0.60, 300: 0.70},
    "left_rear_overtake_stern": {50: 0.38, 100: 0.41, 200: 0.50, 300: 0.60},
    "left_rear_being_overtaken_bow": {50: 0.39, 100: 0.42, 200: 0.50, 300: 0.62},
    "left_rear_being_overtaken_stern": {50: 0.33, 100: 0.37, 200: 0.45, 300: 0.57},
    "right_rear_being_overtaken_bow": {50: 0.39, 100: 0.42, 200: 0.50, 300: 0.62},
    "right_rear_being_overtaken_stern": {50: 0.33, 100: 0.37, 200: 0.45, 300: 0.57},
}

# B.4 relative coefficients for target ship type in relation to a bulk carrier.
SDA_TYPE_COEFFICIENTS = {
    "LNG_or_chemical": 1.5,
    "container": 1.33,
    "passenger": 1.2,
    "tanker": 1.36,
    "other": 1.0,
}


def lookup_annex_b_sda_nmi(encounter_key: str, target_length_m: int) -> float | None:
    """Return exact listed value or None when unsupported (no interpolation)."""
    row = SDA_B1_100M_BULK.get(encounter_key)
    if row is None:
        return None
    return row.get(int(target_length_m))


def lookup_annex_b_sda_m(encounter_key: str, target_length_m: int) -> float | None:
    value = lookup_annex_b_sda_nmi(encounter_key, target_length_m)
    return value * NM_TO_M if value is not None else None
