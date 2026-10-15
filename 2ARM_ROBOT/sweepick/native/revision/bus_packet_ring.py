"""Bounded, observational tracing for the installed Feetech transport.

Install this module before constructing any ``FeetechMotorsBus``.  It never
opens a port or performs I/O of its own; it only wraps calls made by LeRobot
and the installed SCServo SDK.
"""

from __future__ import annotations

import hashlib
import json
import os
import threading
import time
import traceback
from collections import Counter, deque
from pathlib import Path
from typing import Any


_RING_SIZE = 32
_FOLLOWING = 4
_RAW_LIMIT = 4096
_IO_EVENT_LIMIT = 64
_LOCKS: dict[str, threading.RLock] = {}
_LOCKS_GUARD = threading.Lock()
_ACTIVE: Recorder | None = None


def _port_lock(realpath: str) -> threading.RLock:
    with _LOCKS_GUARD:
        return _LOCKS.setdefault(realpath, threading.RLock())


def _bytes(value: Any) -> bytes:
    try:
        return bytes(value)
    except (TypeError, ValueError):
        return b""


def _stack() -> list[dict[str, Any]]:
    frames = traceback.extract_stack()[:-2]
    return [
        {"file": frame.filename, "line": frame.lineno, "function": frame.name}
        for frame in frames[-16:]
        if Path(frame.filename).name != Path(__file__).name
    ]


def _frames(raw: bytes) -> list[dict[str, Any]]:
    """Describe complete or truncated SDK frames without claiming a parse."""
    found: list[dict[str, Any]] = []
    pos = 0
    while pos + 1 < len(raw):
        start = raw.find(b"\xff\xff", pos)
        if start < 0:
            break
        available = len(raw) - start
        frame: dict[str, Any] = {
            "offset": start,
            "header_hex": raw[start : start + 2].hex(),
            "header_valid": True,
            "available_length": available,
        }
        if available >= 3:
            frame["id"] = raw[start + 2]
        if available >= 4:
            declared = raw[start + 3]
            total = declared + 4
            frame.update(declared_length=declared, expected_total_length=total)
            if declared < 2:
                frame.update(malformed=True, complete=False, raw_hex=raw[start : start + max(4, total)].hex())
                pos = start + 2
            elif available >= total:
                packet = raw[start : start + total]
                expected = (~sum(packet[2:-1])) & 0xFF
                frame.update(
                    complete=True,
                    parser_valid=True,
                    error=raw[start + 4],
                    checksum_expected=expected,
                    checksum_actual=packet[-1],
                    checksum_valid=packet[-1] == expected,
                    raw_hex=packet.hex(),
                )
                pos = start + total
            else:
                frame.update(complete=False, raw_hex=raw[start:].hex())
                if available >= 5:
                    frame["error_unverified"] = raw[start + 4]
                pos = len(raw)
        else:
            frame.update(complete=False, raw_hex=raw[start:].hex())
            pos = len(raw)
        found.append(frame)
    return found


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return repr(value)


class Recorder:
    def __init__(self, output_dir: str | os.PathLike[str]):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.ring_path = self.output_dir / "BUS_PACKET_RING.jsonl"
        self.summary_path = self.output_dir / "BUS_DIAG_SUMMARY.json"
        self._ring: deque[dict[str, Any]] = deque(maxlen=_RING_SIZE)
        self._guard = threading.RLock()
        self._local = threading.local()
        self._next_id = 0
        self._written: set[int] = set()
        self._following = 0
        self._first_stack: list[dict[str, Any]] | None = None
        self._counts: Counter[str] = Counter()
        self._errors: Counter[str] = Counter()
        self._latency_count = 0
        self._latency_sum = 0
        self._latency_min: int | None = None
        self._latency_max: int | None = None
        self._logging_errors: Counter[str] = Counter()
        self._logging_error_examples: deque[str] = deque(maxlen=8)
        self._lost_records = 0
        self._ring_evictions = 0
        self._flushed = False

    def _note_logging_error(self, exc: BaseException, lost: int = 0) -> None:
        try:
            with self._guard:
                name = type(exc).__name__
                self._logging_errors[name] += 1
                self._logging_error_examples.append(f"{name}: {exc}")
                self._lost_records += lost
        except BaseException:
            pass

    def _contexts(self) -> dict[int, dict[str, Any]]:
        contexts = getattr(self._local, "contexts", None)
        if contexts is None:
            contexts = self._local.contexts = {}
        return contexts

    def _current(self, port: Any) -> dict[str, Any] | None:
        return self._contexts().get(id(port))

    def _begin(self, bus: Any, kind: str, args: tuple[Any, ...], kwargs: dict[str, Any]) -> dict[str, Any]:
        port = bus.port_handler
        port_name = str(getattr(port, "port_name", bus.port))
        realpath = os.path.realpath(port_name)
        with self._guard:
            self._next_id += 1
            transaction_id = self._next_id
            if self._first_stack is None:
                self._first_stack = _stack()
        names = ("address", "length", "expected_id", "value")
        expected = {name: _jsonable(value) for name, value in zip(names, args)}
        if kind == "sync_read" and len(args) >= 3:
            expected = {
                "address": _jsonable(args[0]),
                "length": _jsonable(args[1]),
                "expected_ids": _jsonable(args[2]),
            }
        elif kind == "sync_write" and len(args) >= 3:
            expected = {
                "address": _jsonable(args[0]),
                "length": _jsonable(args[1]),
                "expected_ids": _jsonable(list(args[2])),
                "value": _jsonable(args[2]),
            }
        side_name = Path(port_name).name.lower()
        side = "left" if "left" in side_name else "right" if "right" in side_name else None
        txn: dict[str, Any] = {
            "transaction_id": transaction_id,
            "kind": kind,
            "monotonic_start_ns": time.monotonic_ns(),
            "pid": os.getpid(),
            "thread_id": threading.get_ident(),
            "thread_name": threading.current_thread().name,
            "port": port_name,
            "port_realpath": realpath,
            "side": side,
            "expected": expected,
            "requested_retries": int(kwargs.get("num_retry", 0)),
            "tx": [],
            "rx_raw": bytearray(),
            "rx_bytes_dropped": 0,
            "empty_read_polls": 0,
            "read_calls": 0,
            "clear_port_calls": 0,
            "timeouts": [],
            "rx_chunks": [],
            "rx_bytes_seen": 0,
            "sdk_rx_results": [],
        }
        if kind == "sync_write":
            txn["response_expected"] = False
        self._contexts()[id(port)] = txn
        return txn

    def _classify(self, txn: dict[str, Any]) -> str | None:
        frames = txn["parser_frames"]
        expected = txn["expected"].get("expected_id")
        expected_ids = txn["expected"].get("expected_ids", [])
        expected_set = set(expected_ids if isinstance(expected_ids, list) else [expected_ids])
        if expected is not None:
            expected_set.add(expected)
        if any(item.get("actual") != item.get("requested") for item in txn["tx"]):
            return "TX_FAIL"
        results = [item.get("result") for item in txn["sdk_rx_results"]]
        comm = txn.get("comm_result")
        if comm is None and results:
            comm = results[-1]
        complete = [frame for frame in frames if frame.get("parser_valid")]
        if any(frame.get("checksum_valid") is False for frame in complete):
            return "RX_CHECKSUM"
        mismatched = [frame for frame in complete if expected_set and frame.get("id") not in expected_set]
        if mismatched:
            return "RX_WRONG_ID"
        matching = [frame for frame in complete if not expected_set or frame.get("id") in expected_set]
        if comm == 0 and any(frame.get("checksum_valid") and frame.get("error", 0) for frame in matching):
            return "SERVO_ERROR"
        if comm == -1:
            return "PORT_BUSY"
        if comm == -2:
            return "TX_FAIL"
        if comm == -3:
            return "RX_FAIL"
        if comm == -4:
            return "TX_ERROR"
        if comm == -5:
            return "RX_WAITING"
        if comm == -7:
            if any(not frame.get("complete", False) for frame in frames):
                return "RX_TRUNCATED"
            return "RX_CORRUPT"
        if comm == -6:
            return "RX_TIMEOUT"
        if txn.get("exception"):
            return "EXCEPTION"
        return None

    def _finish(self, port: Any, txn: dict[str, Any], result: Any = None, exception: BaseException | None = None) -> None:
        self._contexts().pop(id(port), None)
        txn.pop("_packet_deadline_ns", None)
        txn.pop("_tail_capture", None)
        ended = time.monotonic_ns()
        raw = bytes(txn.pop("rx_raw"))
        txn.update(
            monotonic_end_ns=ended,
            latency_ns=ended - txn["monotonic_start_ns"],
            rx_hex=raw.hex(),
            rx_total_bytes=len(raw) + txn["rx_bytes_dropped"],
            parser_frames=_frames(raw),
            result=_jsonable(result),
        )
        if exception is not None:
            txn["exception"] = {"type": type(exception).__name__, "message": str(exception)}
        if result is not None:
            if txn["kind"] == "packet" and isinstance(result, tuple) and len(result) >= 3:
                txn["comm_result"], txn["device_error_result"] = result[1], result[2]
            elif txn["kind"] == "write" and isinstance(result, tuple) and len(result) >= 2:
                txn["comm_result"], txn["device_error_result"] = result[0], result[1]
            elif txn["kind"] == "read" and isinstance(result, tuple) and len(result) >= 3:
                txn["comm_result"], txn["device_error_result"] = result[1], result[2]
            elif txn["kind"] == "sync_read" and isinstance(result, tuple) and len(result) >= 2:
                txn["comm_result"] = result[1]
            elif txn["kind"] == "sync_write" and isinstance(result, int):
                txn["comm_result"] = result
        txn["actual_attempts"] = len(txn["tx"])
        txn["actual_retries"] = max(0, len(txn["tx"]) - 1)
        txn["error_kind"] = self._classify(txn)
        if txn["kind"] == "write":
            ok = txn["error_kind"] is None and isinstance(result, tuple) and result == (0, 0)
            txn["ack_status"] = "CONFIRMED" if ok else "UNKNOWN"
            txn["application_status"] = "UNKNOWN"
        elif txn["kind"] == "sync_write":
            txn["ack_status"] = "NOT_EXPECTED"
            txn["application_status"] = "UNKNOWN"
        if txn["error_kind"]:
            txn["call_stack"] = _stack()
        elif txn["transaction_id"] == 1:
            txn["call_stack"] = self._first_stack
        with self._guard:
            self._counts[f"workload_{txn['kind']}"] += 1
            self._counts["transactions"] += 1
            self._counts["tx_write_calls"] += len(txn["tx"])
            self._counts["rx_read_calls"] += txn["read_calls"]
            self._counts["empty_read_polls"] += txn["empty_read_polls"]
            self._counts["clear_port_calls"] += txn["clear_port_calls"]
            self._counts["actual_retries"] += txn["actual_retries"]
            latency = txn["latency_ns"]
            self._latency_count += 1
            self._latency_sum += latency
            self._latency_min = latency if self._latency_min is None else min(self._latency_min, latency)
            self._latency_max = latency if self._latency_max is None else max(self._latency_max, latency)
            if len(self._ring) == self._ring.maxlen:
                self._ring_evictions += 1
            self._ring.append(txn)
            if txn["error_kind"]:
                self._errors[txn["error_kind"]] += 1
                self._write_unwritten(list(self._ring))
                self._following = max(self._following, _FOLLOWING)
            elif self._following:
                self._write_unwritten([txn])
                self._following -= 1
            live_ids = {item["transaction_id"] for item in self._ring}
            self._written.intersection_update(live_ids)

    def _write_unwritten(self, transactions: list[dict[str, Any]]) -> None:
        pending = [txn for txn in transactions if txn["transaction_id"] not in self._written]
        if not pending:
            return
        try:
            with self.ring_path.open("a", encoding="utf-8") as stream:
                for txn in pending:
                    stream.write(json.dumps(txn, sort_keys=True, separators=(",", ":")) + "\n")
                    self._written.add(txn["transaction_id"])
        except BaseException as exc:
            self._note_logging_error(exc)

    def _safe_finish(
        self,
        port: Any,
        txn: dict[str, Any],
        result: Any = None,
        exception: BaseException | None = None,
    ) -> None:
        try:
            self._finish(port, txn, result=result, exception=exception)
        except BaseException as exc:
            self._contexts().pop(id(port), None)
            self._note_logging_error(exc, lost=1)

    def _capture_failure_tail(self, port: Any, txn: dict[str, Any]) -> None:
        deadline = txn.get("_packet_deadline_ns")
        if not isinstance(deadline, int):
            return
        started = time.monotonic_ns()
        raw = bytearray()
        tail: dict[str, Any] = {
            "label": "post_failure_diagnostic_only",
            "monotonic_start_ns": started,
            "original_timeout_deadline_ns": deadline,
            "read_calls": 0,
            "empty_read_polls": 0,
            "chunks": [],
            "bytes_dropped": 0,
        }
        txn["_tail_capture"] = True
        try:
            while time.monotonic_ns() < deadline and len(txn["rx_raw"]) + len(raw) < _RAW_LIMIT:
                room = _RAW_LIMIT - len(txn["rx_raw"]) - len(raw)
                value = port.readPort(room)
                tail["read_calls"] += 1
                chunk = _bytes(value)
                if not chunk:
                    tail["empty_read_polls"] += 1
                    time.sleep(0)
                    continue
                kept = chunk[:room]
                raw.extend(kept)
                tail["bytes_dropped"] += len(chunk) - len(kept)
                if len(tail["chunks"]) < _IO_EVENT_LIMIT:
                    tail["chunks"].append({
                        "monotonic_ns": time.monotonic_ns(),
                        "request_length": room,
                        "actual_length": len(chunk),
                        "raw_hex": kept.hex(),
                    })
        except BaseException as exc:
            tail["io_exception"] = {"type": type(exc).__name__, "message": str(exc)}
            self._note_logging_error(exc)
        finally:
            txn.pop("_tail_capture", None)
            tail.update(
                monotonic_end_ns=time.monotonic_ns(),
                raw_hex=bytes(raw).hex(),
                total_bytes=len(raw) + tail["bytes_dropped"],
            )
            txn["rx_failure_tail"] = tail

    def _wrap_transaction(self, bus: Any, name: str, kind: str) -> None:
        original = getattr(bus, name)
        setattr(bus, f"_bus_packet_ring_original_{name}", original)

        def wrapped(*args: Any, **kwargs: Any) -> Any:
            port = bus.port_handler
            realpath = os.path.realpath(str(getattr(port, "port_name", bus.port)))
            with _port_lock(realpath):
                try:
                    txn = self._begin(bus, kind, args, kwargs)
                except BaseException as log_exc:
                    self._note_logging_error(log_exc, lost=1)
                    return original(*args, **kwargs)
                try:
                    result = original(*args, **kwargs)
                except BaseException as exc:
                    if (kind == "sync_read" and isinstance(exc, ConnectionError)
                            and str(exc).startswith("response_contract:")):
                        try:
                            self._capture_failure_tail(port, txn)
                        except BaseException as log_exc:
                            self._note_logging_error(log_exc)
                    self._safe_finish(port, txn, exception=exc)
                    raise
                self._safe_finish(port, txn, result=result)
                return result

        setattr(bus, name, wrapped)

    def _attach_bus(self, bus: Any) -> None:
        port = bus.port_handler
        for name in ("readPort", "writePort", "clearPort", "setPacketTimeout", "setPacketTimeoutMillis"):
            original = getattr(port, name)
            setattr(port, f"_bus_packet_ring_original_{name}", original)

            if name == "readPort":
                def wrapped(length: int, _original: Any = original) -> Any:
                    value = _original(length)
                    try:
                        txn = self._current(port)
                        if txn is not None and not txn.get("_tail_capture"):
                            raw = _bytes(value)
                            txn["read_calls"] += 1
                            if raw:
                                txn["rx_bytes_seen"] += len(raw)
                                if len(txn["rx_chunks"]) < _IO_EVENT_LIMIT:
                                    txn["rx_chunks"].append({
                                        "monotonic_ns": time.monotonic_ns(),
                                        "request_length": length,
                                        "actual_length": len(raw),
                                        "cumulative_rx_bytes": txn["rx_bytes_seen"],
                                        "raw_hex": raw.hex(),
                                    })
                                room = max(0, _RAW_LIMIT - len(txn["rx_raw"]))
                                txn["rx_raw"].extend(raw[:room])
                                txn["rx_bytes_dropped"] += max(0, len(raw) - room)
                            else:
                                txn["empty_read_polls"] += 1
                    except BaseException as exc:
                        self._note_logging_error(exc)
                    return value
            elif name == "writePort":
                def wrapped(packet: Any, _original: Any = original) -> Any:
                    actual = _original(packet)
                    try:
                        txn = self._current(port)
                        if txn is not None and len(txn["tx"]) < _IO_EVENT_LIMIT:
                            txn["tx"].append({
                                "monotonic_ns": time.monotonic_ns(),
                                "raw_hex": _bytes(packet).hex(),
                                "requested": len(packet),
                                "actual": _jsonable(actual),
                            })
                    except BaseException as exc:
                        self._note_logging_error(exc)
                    return actual
            elif name == "clearPort":
                def wrapped(_original: Any = original) -> Any:
                    value = _original()
                    try:
                        txn = self._current(port)
                        if txn is not None:
                            txn["clear_port_calls"] += 1
                    except BaseException as exc:
                        self._note_logging_error(exc)
                    return value
            else:
                timeout_kind = "packet_length" if name == "setPacketTimeout" else "milliseconds"

                def wrapped(value: Any, _original: Any = original, _kind: str = timeout_kind) -> Any:
                    timeout_started_ns = time.monotonic_ns()
                    result = _original(value)
                    try:
                        txn = self._current(port)
                        if txn is not None:
                            actual_timeout = getattr(port, "packet_timeout", None)
                            if len(txn["timeouts"]) < _IO_EVENT_LIMIT:
                                txn["timeouts"].append({
                                    "kind": _kind,
                                    "value": _jsonable(value),
                                    "actual_packet_timeout": _jsonable(actual_timeout),
                                })
                            try:
                                timeout_ns = int(float(actual_timeout) * 1_000_000)
                            except (TypeError, ValueError, OverflowError):
                                txn.pop("_packet_deadline_ns", None)
                            else:
                                if timeout_ns >= 0:
                                    txn["_packet_deadline_ns"] = timeout_started_ns + timeout_ns
                                else:
                                    txn.pop("_packet_deadline_ns", None)
                    except BaseException as exc:
                        self._note_logging_error(exc)
                    return result
            setattr(port, name, wrapped)

        packet = bus.packet_handler
        original_rx = packet.rxPacket
        setattr(packet, "_bus_packet_ring_original_rxPacket", original_rx)

        def rx_packet(wrapped_port: Any) -> Any:
            result = original_rx(wrapped_port)
            try:
                txn = self._current(wrapped_port)
                if txn is not None and len(txn["sdk_rx_results"]) < _IO_EVENT_LIMIT:
                    rxpacket, comm = result
                    txn["sdk_rx_results"].append({"result": _jsonable(comm), "packet_hex": _bytes(rxpacket).hex()})
            except BaseException as exc:
                self._note_logging_error(exc)
            return result

        packet.rxPacket = rx_packet
        original_txrx = packet.txRxPacket
        setattr(packet, "_bus_packet_ring_original_txRxPacket", original_txrx)

        def txrx_packet(wrapped_port: Any, txpacket: Any) -> Any:
            # Public bus operations already own the transaction.  This fallback
            # covers handshake ping/model reads and direct SDK transactions.
            if self._current(wrapped_port) is not None:
                return original_txrx(wrapped_port, txpacket)
            raw = list(txpacket)
            expected_id = raw[2] if len(raw) > 2 else None
            instruction = raw[4] if len(raw) > 4 else None
            address = raw[5] if instruction in (2, 3, 4) and len(raw) > 5 else None
            value = raw[6:-1] if instruction in (3, 4) and len(raw) > 6 else None
            if expected_id == 0xFE:
                expected_response_length = None
            elif instruction == 2 and len(raw) > 6:
                expected_response_length = 6 + raw[6]
            else:
                expected_response_length = 6
            realpath = os.path.realpath(str(getattr(wrapped_port, "port_name", bus.port)))
            with _port_lock(realpath):
                try:
                    txn = self._begin(bus, "packet", (address, expected_response_length, expected_id, value), {})
                    txn["expected"]["instruction"] = instruction
                except BaseException as log_exc:
                    self._note_logging_error(log_exc, lost=1)
                    return original_txrx(wrapped_port, txpacket)
                try:
                    result = original_txrx(wrapped_port, txpacket)
                except BaseException as exc:
                    self._safe_finish(wrapped_port, txn, exception=exc)
                    raise
                self._safe_finish(wrapped_port, txn, result=result)
                return result

        packet.txRxPacket = txrx_packet
        self._wrap_transaction(bus, "_read", "read")
        self._wrap_transaction(bus, "_write", "write")
        self._wrap_transaction(bus, "_sync_read", "sync_read")
        self._wrap_transaction(bus, "_sync_write", "sync_write")

    def summary(self) -> dict[str, Any]:
        with self._guard:
            source = Path(__file__)
            result: dict[str, Any] = {
                "schema": "tjj.bus-packet-ring.v1",
                "pid": os.getpid(),
                "hook": {"path": str(source.resolve()), "sha256": hashlib.sha256(source.read_bytes()).hexdigest()},
                "software_bounds": {
                    "ring_transactions": _RING_SIZE,
                    "following_transactions": _FOLLOWING,
                    "raw_bytes_per_transaction": _RAW_LIMIT,
                },
                "counts": dict(self._counts),
                "errors": dict(self._errors),
                "logging": {
                    "errors": dict(self._logging_errors),
                    "examples": list(self._logging_error_examples),
                    "lost_records": self._lost_records,
                    "bounded_ring_evictions": self._ring_evictions,
                    "unwritten_ring_records": sum(
                        item["transaction_id"] not in self._written for item in self._ring
                    ),
                },
                "first_transaction_call_stack": self._first_stack,
            }
            if self._latency_count:
                result["response_latency_ns"] = {
                    "count": self._latency_count,
                    "min": self._latency_min,
                    "max": self._latency_max,
                    "mean": self._latency_sum / self._latency_count,
                }
            else:
                result["response_latency_ns"] = {"count": 0, "min": None, "max": None, "mean": None}
            try:
                import importlib.metadata
                import scservo_sdk

                sdk_source = Path(scservo_sdk.__file__).resolve()
                result["sdk"] = {
                    "package": "feetech-servo-sdk",
                    "version": importlib.metadata.version("feetech-servo-sdk"),
                    "path": str(sdk_source),
                    "sha256": hashlib.sha256(sdk_source.read_bytes()).hexdigest(),
                }
            except (ImportError, OSError, importlib.metadata.PackageNotFoundError) as exc:
                result["sdk"] = {"identity_error": f"{type(exc).__name__}: {exc}"}
            return result

    def flush(self) -> dict[str, Any]:
        with self._guard:
            self._write_unwritten(list(self._ring))
            result = self.summary()
            try:
                self.summary_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            except BaseException as exc:
                self._note_logging_error(exc)
                result = self.summary()
            self._flushed = True
            return result


def install(output_dir: str | os.PathLike[str]) -> Recorder:
    """Install the constructor hook and return its recorder."""
    global _ACTIVE
    from lerobot.motors.feetech.feetech import FeetechMotorsBus

    recorder = Recorder(output_dir)
    _ACTIVE = recorder
    if not getattr(FeetechMotorsBus.__init__, "_bus_packet_ring_hook", False):
        original_init = FeetechMotorsBus.__init__

        def hooked_init(self: Any, *args: Any, **kwargs: Any) -> None:
            original_init(self, *args, **kwargs)
            active = _ACTIVE
            if active is not None:
                active._attach_bus(self)

        hooked_init._bus_packet_ring_hook = True  # type: ignore[attr-defined]
        hooked_init._bus_packet_ring_original = original_init  # type: ignore[attr-defined]
        FeetechMotorsBus.__init__ = hooked_init
    return recorder
