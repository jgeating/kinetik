"""
NTHelper.py - Python port of NTHelper.java

Convenience wrappers around the RobotPy ntcore API for reading and writing
typed values to NetworkTables.  All functions operate on the default
NetworkTableInstance, mirroring the behaviour of the original Java class.

Dependencies: robotpy-ntcore  (pip install robotpy-ntcore)
"""

from __future__ import annotations

from typing import Callable, List

import ntcore


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
    """Write *value* as a double to *key*."""
    get_entry(key).setDouble(value)


# ---------------------------------------------------------------------------
# str
# ---------------------------------------------------------------------------

def get_string(key: str, default_value: str) -> str:
    """Return the string stored at *key*, or *default_value* if absent."""
    return get_entry(key).getString(default_value)


def set_string(key: str, value: str) -> None:
    """Write *value* as a string to *key*."""
    get_entry(key).setString(value)


# ---------------------------------------------------------------------------
# bool
# ---------------------------------------------------------------------------

def get_boolean(key: str, default_value: bool) -> bool:
    """Return the boolean stored at *key*, or *default_value* if absent."""
    return get_entry(key).getBoolean(default_value)


def set_boolean(key: str, value: bool) -> None:
    """Write *value* as a boolean to *key*."""
    get_entry(key).setBoolean(value)


# ---------------------------------------------------------------------------
# List[str]
# ---------------------------------------------------------------------------

def get_string_array(key: str, default_value: List[str]) -> List[str]:
    """Return the string array stored at *key*, or *default_value* if absent."""
    return list(get_entry(key).getStringArray(default_value))


def set_string_array(key: str, value: List[str]) -> None:
    """Write *value* as a string array to *key*."""
    get_entry(key).setStringArray(value)


# ---------------------------------------------------------------------------
# List[bool]
# ---------------------------------------------------------------------------

def get_boolean_array(key: str, default_value: List[bool]) -> List[bool]:
    """Return the boolean array stored at *key*, or *default_value* if absent."""
    return list(get_entry(key).getBooleanArray(default_value))


def set_boolean_array(key: str, value: List[bool]) -> None:
    """Write *value* as a boolean array to *key*."""
    get_entry(key).setBooleanArray(value)


# ---------------------------------------------------------------------------
# List[float]
# ---------------------------------------------------------------------------

def get_double_array(key: str, default_value: List[float]) -> List[float]:
    """Return the double array stored at *key*, or *default_value* if absent."""
    return list(get_entry(key).getDoubleArray(default_value))


def set_double_array(key: str, value: List[float]) -> None:
    """Write *value* as a double array to *key*."""
    get_entry(key).setDoubleArray(value)
