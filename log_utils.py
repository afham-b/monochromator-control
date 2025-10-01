# log_utils.py
import os
import logging

__all__ = [
    "TailTruncatingFileHandler",
    "enforce_log_size_limit",
]

class TailTruncatingFileHandler(logging.FileHandler):
    """
    Keeps the log file under `max_bytes` by retaining only the last `keep_bytes`
    of content when the file grows too large. Truncates from the head and aligns
    to a newline to avoid partial lines.
    """
    def __init__(self, filename, mode='a', encoding=None, delay=False,
                 max_bytes=10 * 1024 * 1024, keep_bytes=None, check_every=20):
        super().__init__(filename, mode, encoding, delay)
        self.baseFilename = os.fspath(filename)
        self.max_bytes = int(max_bytes)
        self.keep_bytes = int(keep_bytes if keep_bytes is not None else max_bytes * 0.8)
        self.check_every = max(1, int(check_every))  # check once per N emits
        self._emit_counter = 0

    def emit(self, record):
        try:
            self._emit_counter += 1
            if self._emit_counter % self.check_every == 0:
                self._maybe_truncate()
            super().emit(record)
        except Exception:
            self.handleError(record)

    def _maybe_truncate(self):
        try:
            if self.stream:
                self.stream.flush()
            size = os.path.getsize(self.baseFilename)
            if size <= self.max_bytes:
                return
        except FileNotFoundError:
            return

        # Close stream before rewriting the file (Windows-safe)
        if self.stream:
            try:
                self.stream.flush()
            except Exception:
                pass
            try:
                self.stream.close()
            except Exception:
                pass
            self.stream = None

        path = self.baseFilename
        size = os.path.getsize(path)
        keep = min(self.keep_bytes, size)

        with open(path, 'rb') as f:
            f.seek(-keep, os.SEEK_END)
            chunk = f.read()

        # Align to a newline so the new first line is whole
        nl = chunk.find(b'\n')
        if nl != -1 and nl + 1 < len(chunk):
            chunk = chunk[nl+1:]

        tmp = path + '.tmp'
        with open(tmp, 'wb') as out:
            out.write(chunk)
        os.replace(tmp, path)

        # Reopen so logging continues seamlessly
        self._open()


def enforce_log_size_limit(path, max_bytes=10*1024*1024, keep_bytes=None):
    """
    One-shot trim you can call at startup: keeps only the last `keep_bytes`
    (default 80% of max_bytes) and aligns to a newline.
    """
    keep = int(keep_bytes if keep_bytes is not None else max_bytes * 0.8)
    try:
        size = os.path.getsize(path)
    except FileNotFoundError:
        return
    if size <= max_bytes:
        return

    with open(path, 'rb') as f:
        f.seek(-min(keep, size), os.SEEK_END)
        chunk = f.read()

    nl = chunk.find(b'\n')
    if nl != -1 and nl + 1 < len(chunk):
        chunk = chunk[nl+1:]

    tmp = path + '.tmp'
    with open(tmp, 'wb') as out:
        out.write(chunk)
    os.replace(tmp, path)
