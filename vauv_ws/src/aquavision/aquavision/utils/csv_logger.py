"""
CSV Logger utility for Aquavision 2.0.

Thread-safe CSV logger that writes mission telemetry to timestamped files
in ~/aquavision_logs/.
"""

import csv
import os
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


# Column headers for the mission log
LOG_COLUMNS: List[str] = [
    'timestamp',
    'state',
    'error_x',
    'error_y',
    'width_ratio',
    'thrust_x',
    'thrust_y',
    'thrust_z',
    'thrust_r',
    'depth',
]


class CSVLogger:
    """Thread-safe CSV logger for mission telemetry.

    Creates a timestamped CSV file in ~/aquavision_logs/ and provides
    a thread-safe interface for logging rows of mission data.

    Attributes:
        filepath: Absolute path to the log file being written.
    """

    def __init__(self, log_dir: Optional[str] = None) -> None:
        """Initialize the CSV logger.

        Args:
            log_dir: Directory to write log files. Defaults to
                     ~/aquavision_logs/.
        """
        if log_dir is None:
            log_dir = os.path.join(Path.home(), 'aquavision_logs')

        os.makedirs(log_dir, exist_ok=True)

        timestamp_str: str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename: str = f'mission_{timestamp_str}.csv'
        self.filepath: str = os.path.join(log_dir, filename)

        self._lock: threading.Lock = threading.Lock()
        self._file = open(self.filepath, 'w', newline='')
        self._writer: csv.writer = csv.writer(self._file)
        self._writer.writerow(LOG_COLUMNS)
        self._file.flush()

    def log_row(
        self,
        state: str,
        error_x: float = 0.0,
        error_y: float = 0.0,
        width_ratio: float = 0.0,
        thrust_x: float = 0.0,
        thrust_y: float = 0.0,
        thrust_z: float = 0.0,
        thrust_r: float = 0.0,
        depth: float = 0.0,
    ) -> None:
        """Log a single row of mission telemetry.

        Thread-safe. Flushes after each write for crash resilience.

        Args:
            state: Current mission state name.
            error_x: Normalized horizontal error.
            error_y: Normalized vertical error.
            width_ratio: Gate bounding box width ratio.
            thrust_x: Forward/backward thrust command.
            thrust_y: Lateral thrust command.
            thrust_z: Vertical thrust command.
            thrust_r: Yaw thrust command.
            depth: Current depth (if available).
        """
        timestamp: str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        row: list = [
            timestamp,
            state,
            f'{error_x:.4f}',
            f'{error_y:.4f}',
            f'{width_ratio:.4f}',
            f'{thrust_x:.1f}',
            f'{thrust_y:.1f}',
            f'{thrust_z:.1f}',
            f'{thrust_r:.1f}',
            f'{depth:.2f}',
        ]

        with self._lock:
            self._writer.writerow(row)
            self._file.flush()

    def close(self) -> None:
        """Close the log file. Safe to call multiple times."""
        with self._lock:
            if self._file and not self._file.closed:
                self._file.flush()
                self._file.close()

    def __del__(self) -> None:
        """Ensure file is closed on garbage collection."""
        try:
            self.close()
        except Exception:
            pass
