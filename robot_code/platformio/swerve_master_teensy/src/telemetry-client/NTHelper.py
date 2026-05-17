"""
NTHelper.py - Python port of NTHelper.java

Convenience wrappers around the RobotPy ntcore API for reading and writing
typed values to NetworkTables.  All functions operate on the default
NetworkTableInstance, mirroring the behaviour of the original Java class.

Every set_* call also appends the value to a local .wpilog data log so that
the full time-series is captured automatically.  The log file is created in
the working directory and named telemetry_YYYYMMDD_HHMMSS.wpilog.

Dependencies: robotpy-ntcore  (pip install robotpy-ntcore)
             robotpy-wpiutil   (pip install robotpy-wpiutil)
"""

from __future__ import annotations

import datetime
from typing import Callable, List, Type

import ntcore
from wpiutil import DataLogBackgroundWriter
from wpiutil.log import (
    BooleanLogEntry,
    BooleanArrayLogEntry,
    DoubleLogEntry,
    DoubleArrayLogEntry,
    StringLogEntry,
    StringArrayLogEntry,
)


# ---------------------------------------------------------------------------
# Data log
# ---------------------------------------------------------------------------
_log_filename = datetime.datetime.now().strftime("telemetry_%Y%m%d_%H%M%S.wpilog")
_datalog = DataLogBackgroundWriter(dir=".", filename=_log_filename)

# Cache of key → LogEntry so each entry is only created once.
_log_entries: dict = {}


def _log(key: str, value, entry_class: Type) -> None:
    """Append *value* to the typed log entry for *key*, creating it if needed."""
    if key not in _log_entries:
        _log_entries[key] = entry_class(_datalog, key)
    _log_entries[key].append(value)


# ---------------------------------------------------------------------------
# Instance / entry helpers
# ---------------------------------------------------------------------------

def get_instance() -> ntcore.NetworkTableInstance:
    """Return the default NetworkTableInstance."""
    return ntcore.NetworkTableInstance.getDefault()


def get_entry(key: str) -> ntcore.NetworkTableEntry:
    """Return the NetworkTableEntry for *key*."""
    return get_instance().getEntry(key)


# ---------------------------------------------------------------------------
# Persistence
# ---------------------------------------------------------------------------

def set_persistent(key: str) -> None:
    """Mark the entry at *key* as persistent (survives robot restarts)."""
    get_entry(key).setPersistent()


# ---------------------------------------------------------------------------
# Listener
# ---------------------------------------------------------------------------

def listen(key: str, listener: Callable[[ntcore.Event], None]) -> None:
    """
    Register a callback that fires whenever the value at *key* changes.

    :param key:      NetworkTables key to monitor.
    :param listener: Callable that receives an :class:`ntcore.Event` on change.
    """
    entry = get_entry(key)
    get_instance().addListener(entry, ntcore.EventFlags.kValueAll, listener)


# ---------------------------------------------------------------------------
# double
# ---------------------------------------------------------------------------

def get_double(key: str, default_value: float) -> float:
    """Return the double stored at *key*, or *default_value* if absent."""
    return get_entry(key).getDouble(default_value)


def set_double(key: str, value: float) -> None:
    """Write *value* as a double to *key* and append it to the data log."""
    get_entry(key).setDouble(value)
    _log(key, value, DoubleLogEntry)


# ---------------------------------------------------------------------------
# str
# ---------------------------------------------------------------------------

def get_string(key: str, default_value: str) -> str:
    """Return the string stored at *key*, or *default_value* if absent."""
    return get_entry(key).getString(default_value)


def set_string(key: str, value: str) -> None:
    """Write *value* as a string to *key* and append it to the data log."""
    get_entry(key).setString(value)
    _log(key, value, StringLogEntry)


# ---------------------------------------------------------------------------
# bool
# ---------------------------------------------------------------------------

def get_boolean(key: str, default_value: bool) -> bool:
    """Return the boolean stored at *key*, or *default_value* if absent."""
    return get_entry(key).getBoolean(default_value)


def set_boolean(key: str, value: bool) -> None:
    """Write *value* as a boolean to *key* and append it to the data log."""
    get_entry(key).setBoolean(value)
    _log(key, value, BooleanLogEntry)


# ---------------------------------------------------------------------------
# List[str]
# ---------------------------------------------------------------------------

def get_string_array(key: str, default_value: List[str]) -> List[str]:
    """Return the string array stored at *key*, or *default_value* if absent."""
    return list(get_entry(key).getStringArray(default_value))


def set_string_array(key: str, value: List[str]) -> None:
    """Write *value* as a string array to *key* and append it to the data log."""
    get_entry(key).setStringArray(value)
    _log(key, value, StringArrayLogEntry)


# ---------------------------------------------------------------------------
# List[bool]
# ---------------------------------------------------------------------------

def get_boolean_array(key: str, default_value: List[bool]) -> List[bool]:
    """Return the boolean array stored at *key*, or *default_value* if absent."""
    return list(get_entry(key).getBooleanArray(default_value))


def set_boolean_array(key: str, value: List[bool]) -> None:
    """Write *value* as a boolean array to *key* and append it to the data log."""
    get_entry(key).setBooleanArray(value)
    _log(key, value, BooleanArrayLogEntry)


# ---------------------------------------------------------------------------
# List[float]
# ---------------------------------------------------------------------------

def get_double_array(key: str, default_value: List[float]) -> List[float]:
    """Return the double array stored at *key*, or *default_value* if absent."""
    return list(get_entry(key).getDoubleArray(default_value))


def set_double_array(key: str, value: List[float]) -> None:
    """Write *value* as a double array to *key* and append it to the data log."""
    get_entry(key).setDoubleArray(value)
    _log(key, value, DoubleArrayLogEntry)
