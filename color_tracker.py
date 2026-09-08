"""
Tankbot Color Tracker (Root-Level Modular Wrapper)
Exports ColorTracker and DetectionResult for easy root imports.
"""

from vision.color_tracker import ColorTracker, DetectionResult

__all__ = ["ColorTracker", "DetectionResult"]
