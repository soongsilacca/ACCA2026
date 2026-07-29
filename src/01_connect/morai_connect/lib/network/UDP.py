import socket
import ctypes
import threading
import queue
import copy

class Receiver:
    def __init__(self, ip, port, data_type):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2**20)  # 1 MB
        self.socket.settimeout(0.5)  # unblock every 0.5s so threads can check _stop
        self.socket.bind((ip, port))

        self.data_type  = data_type
        self.parsed_data = data_type
        self.data_size  = ctypes.sizeof(data_type)

        self._stop  = threading.Event()
        self._queue = queue.Queue(maxsize=10)
        self._lock  = threading.Lock()

        threading.Thread(target=self._recv_loop,  daemon=True, name=f"udp_recv_{port}").start()
        threading.Thread(target=self._parse_loop, daemon=True, name=f"udp_parse_{port}").start()

    # ── private ──────────────────────────────────────────────────
    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                raw_data, _ = self.socket.recvfrom(self.data_size)
            except socket.timeout:
                continue  # check _stop flag and retry
            except OSError:
                break     # socket was closed
            with self._lock:
                ctypes.memmove(ctypes.addressof(self.data_type), raw_data, self.data_size)
                try:
                    self.data_type.parsing()
                except Exception:
                    pass
            # non-blocking put; drop oldest if full to avoid lag
            if self._queue.full():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
            self._queue.put_nowait(self.data_type)

    def _parse_loop(self):
        while not self._stop.is_set():
            try:
                self.parsed_data = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue

    # ── public ───────────────────────────────────────────────────
    def get_data(self):
        return self.parsed_data

    def stop(self):
        """Signal both threads to exit cleanly."""
        self._stop.set()
        self.socket.close()

    def __del__(self):
        self.stop()


class Sender:
    def __init__(self, ip, port):
        self.ip   = ip
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, data):
        packed = ctypes.string_at(ctypes.addressof(data), ctypes.sizeof(data))
        self.socket.sendto(packed, (self.ip, self.port))