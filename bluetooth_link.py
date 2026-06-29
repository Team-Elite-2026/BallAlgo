"""
Robot-to-robot Bluetooth (RFCOMM) link.

Two Pis talk over Bluetooth Classic RFCOMM, which behaves like a UART/serial
pipe — same mental model as the Teensy ASCII protocol, just wireless.

  - The "master" robot runs as the SERVER: it binds an RFCOMM channel and
    waits for the slave to connect.
  - The "slave" robot runs as the CLIENT: it connects to the master's MAC.

All socket work happens on a background thread so the 120 Hz vision loop in
main.py never blocks. The main loop only ever calls:

    bt = BluetoothLink.from_params()      # reads MACs/role from params.json
    bt.start()
    ...
    bt.send(mode, x, y, pred_x, pred_y)   # non-blocking, fire and forget
    peer = bt.get_peer_state()            # latest decoded peer packet (or None)

Wire format (ASCII, newline-terminated), e.g. mode=1, x=2.0, y=-3.0,
pred_x=2.5, pred_y=-3.5:

    "1a2.0x-3.0y2.5p-3.5q\n"

    mode : robot mode   (0 = defense, 1 = offense)
    x,y  : current field-relative robot position
    px,py: next-frame predicted field-relative robot pose
"""

import json
import os
import re
import socket
import threading
import time

# AF_BLUETOOTH / BTPROTO_RFCOMM only exist on Linux (the Pi). On a dev Mac these
# attributes are missing, so we degrade gracefully to a disabled no-op link.
BLUETOOTH_AVAILABLE = hasattr(socket, "AF_BLUETOOTH") and hasattr(socket, "BTPROTO_RFCOMM")

BDADDR_ANY = "00:00:00:00:00:00"

# Matches "1a2.0x-3.0y2.5p-3.5q"
_PACKET_RE = re.compile(
    r"(\d+)a(-?\d+(?:\.\d+)?)x(-?\d+(?:\.\d+)?)y(-?\d+(?:\.\d+)?)p(-?\d+(?:\.\d+)?)q"
)


class PeerState:
    """Latest decoded packet from the other robot."""

    __slots__ = ("mode", "x", "y", "pred_x", "pred_y", "recv_time")

    def __init__(self, mode, x, y, pred_x, pred_y, recv_time):
        self.mode = mode          # int: 0 = defense, 1 = offense
        self.x = x                # float: current field-rel x
        self.y = y                # float: current field-rel y
        self.pred_x = pred_x      # float: predicted next-frame x
        self.pred_y = pred_y      # float: predicted next-frame y
        self.recv_time = recv_time

    def is_fresh(self, max_age_s=0.3):
        return (time.monotonic() - self.recv_time) <= max_age_s

    def __repr__(self):
        return (f"PeerState(mode={self.mode}, x={self.x}, y={self.y}, "
                f"pred=({self.pred_x}, {self.pred_y}))")


def _find_params(path):
    """Resolve params.json from an explicit path or a few common locations."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    if path:
        candidates.append(path)
    candidates += [
        os.path.join(here, "params.json"),
        os.path.join(here, "BallAlgo", "params.json"),
        "params.json",
        os.path.join("BallAlgo", "params.json"),
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return None


class BluetoothLink:
    def __init__(self, role, peer_mac, channel=1, send_hz=30.0, peer_stale_s=0.3):
        """
        role     : "master" (RFCOMM server) or "slave" (RFCOMM client)
        peer_mac : the OTHER robot's Bluetooth MAC (required for the slave;
                   optional for the master, which just listens)
        channel  : RFCOMM channel both robots agree on (1 is fine)
        send_hz  : rate cap for outgoing packets
        """
        self.role = role.lower().strip()
        self.is_master = self.role == "master"
        self.peer_mac = peer_mac
        self.channel = int(channel)
        self.min_send_interval = 1.0 / send_hz if send_hz > 0 else 0.0
        self.peer_stale_s = peer_stale_s

        self.enabled = BLUETOOTH_AVAILABLE
        if not BLUETOOTH_AVAILABLE:
            print("[BT] Bluetooth RFCOMM not available on this platform; link disabled.")
        if not self.is_master and not self.peer_mac:
            print("[BT] Slave role requires peer_mac; link disabled.")
            self.enabled = False

        self._lock = threading.Lock()
        self._sock = None              # active connection socket (or None)
        self._listener = None          # server listening socket (master only)
        self._peer = None              # latest PeerState
        self._running = False
        self._thread = None
        self._last_send = 0.0
        self._rx_buf = b""

    # ------------------------------------------------------------------ #
    # Construction from params.json
    # ------------------------------------------------------------------ #
    @classmethod
    def from_params(cls, path=None):
        params_path = _find_params(path)
        if params_path is None:
            print("[BT] params.json not found; link disabled.")
            return cls(role="master", peer_mac="", channel=1)  # disabled-ish no-op

        with open(params_path, "r") as f:
            params = json.load(f)

        bt = params.get("bluetooth", {})
        if not bt.get("enabled", False):
            link = cls(role=bt.get("role", "master"), peer_mac=bt.get("peer_mac", ""),
                       channel=bt.get("rfcomm_channel", 1))
            link.enabled = False
            print("[BT] bluetooth.enabled is false in params.json; link disabled.")
            return link

        print(f"[BT] Loaded config from {params_path}")
        return cls(
            role=bt.get("role", "master"),
            peer_mac=bt.get("peer_mac", ""),
            channel=bt.get("rfcomm_channel", 1),
            send_hz=bt.get("send_hz", 30.0),
            peer_stale_s=bt.get("peer_stale_s", 0.3),
        )

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #
    def start(self):
        if not self.enabled:
            return
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="bt-link", daemon=True)
        self._thread.start()
        print(f"[BT] Started as {self.role} on RFCOMM channel {self.channel}")

    def stop(self):
        self._running = False
        self._close_connection()
        with self._lock:
            if self._listener is not None:
                try:
                    self._listener.close()
                except OSError:
                    pass
                self._listener = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def connected(self):
        with self._lock:
            return self._sock is not None

    # ------------------------------------------------------------------ #
    # Public send / receive
    # ------------------------------------------------------------------ #
    def send(self, mode, x, y, pred_x, pred_y):
        """Encode and transmit one packet. Non-blocking; rate-limited by send_hz.

        Returns True if the bytes were handed to the socket, False otherwise.
        """
        if not self.enabled:
            return False

        now = time.monotonic()
        if (now - self._last_send) < self.min_send_interval:
            return False

        with self._lock:
            sock = self._sock
        if sock is None:
            return False

        line = self.encode(mode, x, y, pred_x, pred_y)
        try:
            sock.sendall(line)
            self._last_send = now
            return True
        except OSError:
            # Connection broke; let the background thread rebuild it.
            self._close_connection()
            return False

    def get_peer_state(self):
        """Return the latest PeerState if fresh, else None."""
        with self._lock:
            peer = self._peer
        if peer is None or not peer.is_fresh(self.peer_stale_s):
            return None
        return peer

    # ------------------------------------------------------------------ #
    # Encoding / decoding
    # ------------------------------------------------------------------ #
    @staticmethod
    def encode(mode, x, y, pred_x, pred_y):
        return (f"{int(mode)}a{x:.1f}x{y:.1f}y{pred_x:.1f}p{pred_y:.1f}q\n"
                ).encode("ascii")

    @staticmethod
    def decode(text):
        m = _PACKET_RE.search(text)
        if not m:
            return None
        try:
            return PeerState(
                mode=int(m.group(1)),
                x=float(m.group(2)),
                y=float(m.group(3)),
                pred_x=float(m.group(4)),
                pred_y=float(m.group(5)),
                recv_time=time.monotonic(),
            )
        except ValueError:
            return None

    # ------------------------------------------------------------------ #
    # Background connection + receive loop
    # ------------------------------------------------------------------ #
    def _run(self):
        backoff = 0.2
        while self._running:
            try:
                if self._ensure_connected():
                    backoff = 0.2          # reset after a good connection
                    self._receive_loop()
                else:
                    time.sleep(backoff)
                    backoff = min(backoff * 2.0, 2.0)
            except OSError as e:
                print(f"[BT] link error: {e}")
                self._close_connection()
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 2.0)

    def _ensure_connected(self):
        if self.connected():
            return True
        if self.is_master:
            return self._accept_peer()
        return self._connect_to_peer()

    def _accept_peer(self):
        """Master: bind/listen once, then accept (with timeout so we stay responsive)."""
        with self._lock:
            if self._listener is None:
                lst = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                    socket.BTPROTO_RFCOMM)
                lst.bind((BDADDR_ANY, self.channel))
                lst.listen(1)
                self._listener = lst
            listener = self._listener

        listener.settimeout(1.0)
        try:
            conn, addr = listener.accept()
        except socket.timeout:
            return False
        conn.settimeout(1.0)
        with self._lock:
            self._sock = conn
        print(f"[BT] Peer connected from {addr}")
        return True

    def _connect_to_peer(self):
        """Slave: open a fresh socket and connect to the master's MAC."""
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                             socket.BTPROTO_RFCOMM)
        sock.settimeout(5.0)
        try:
            sock.connect((self.peer_mac, self.channel))
        except OSError:
            sock.close()
            return False
        sock.settimeout(1.0)
        with self._lock:
            self._sock = sock
        print(f"[BT] Connected to peer {self.peer_mac}")
        return True

    def _receive_loop(self):
        self._rx_buf = b""
        while self._running and self.connected():
            with self._lock:
                sock = self._sock
            if sock is None:
                break
            try:
                data = sock.recv(512)
            except socket.timeout:
                continue                      # idle; loop to re-check flags
            except OSError:
                break
            if not data:                      # peer closed the connection
                break

            self._rx_buf += data
            while b"\n" in self._rx_buf:
                line, self._rx_buf = self._rx_buf.split(b"\n", 1)
                peer = self.decode(line.decode("ascii", errors="ignore"))
                if peer is not None:
                    with self._lock:
                        self._peer = peer
            # Guard against a peer that never sends newlines.
            if len(self._rx_buf) > 4096:
                self._rx_buf = b""

        self._close_connection()

    def _close_connection(self):
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None


# Quick standalone smoke test:  python3 bluetooth_link.py
if __name__ == "__main__":
    link = BluetoothLink.from_params()
    link.start()
    try:
        while True:
            link.send(mode=1, x=2.0, y=-3.0, pred_x=2.5, pred_y=-3.5)
            peer = link.get_peer_state()
            if peer is not None:
                print("peer:", peer)
            time.sleep(0.1)
    except KeyboardInterrupt:
        link.stop()
