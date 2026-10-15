#!/usr/bin/env python3
"""Configure the-receiver over the serial port (instead of the AP web page).

Dialog (text window):  python configure.py COM3
Line prompts:          python configure.py COM3 --text
Non-interactive:       python configure.py COM3 --wifi-ssid MyNet --wifi-pass secret \\
                           --mqtt-host 192.168.0.10 --channel 2 --sensor-id 1D00 --yes
Requires: pip install pyserial   (Windows dialog also: pip install windows-curses)

The dialog shows the same fields as the AP page with the values stored in the device.
Move with Up/Down or Tab, Enter edits a field, then choose SAVE or EXIT.
With any field option only those fields are changed, without the dialog.
Nothing is written until SAVE; then the device stores the values in EEPROM and
reboots. Close other serial monitors (capture_log.py too) first: the port can be
opened by one program only.

Protocol (lines starting with "CFG ", see the firmware): PING, GET, SET key=value,
SAVE, CANCEL. Opening the port usually resets the board, so the script pings
until the firmware (which may be busy with WiFi/NTP for a while) answers.
"""
import argparse
import getpass
import os
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed. Install it (macOS: inside a virtual environment):\n"
             "  python3 -m venv .venv && source .venv/bin/activate && pip install pyserial")

BOOT_WAIT = 90.0      # max seconds to wait for the firmware to answer a ping
REPLY_TIMEOUT = 5.0
WATCH_SECONDS = 15.0  # how long to show the log after the reboot

# key, label, secret, hint (same order as the AP page)
FIELDS = [
    ("wifi_ssid", "WiFi SSID", False, "required"),
    ("wifi_pass", "WiFi password", True, "Enter a new one; the stored one is never shown"),
    ("mqtt_host", "MQTT host", False, "required"),
    ("mqtt_port", "MQTT port", False, "empty = 1883"),
    ("mqtt_user", "MQTT username", False, ""),
    ("mqtt_pass", "MQTT password", True, "Enter a new one; the stored one is never shown"),
    ("sensor_id", "Sensor ID (hex)", False, "0 = auto; use the stable= value from the log"),
    ("channel", "Sensor channel", False, "1, 2 or 3 (Enter/Space/Left/Right switches)"),
    ("ntp", "NTP server", False, "empty = pl.pool.ntp.org"),
    ("gmt", "GMT offset, hours", False, "empty = Warsaw with DST; or e.g. 1, -5, 5.5"),
]
REQUIRED = ("wifi_ssid", "mqtt_host")
CHOICES = {"channel": ["1", "2", "3"]}
CFG_RE = re.compile(r"CFG (\S+)\s?(.*)$")


class Device:
    def __init__(self, ser):
        self.ser = ser

    def read_cfg(self, timeout):
        """Next 'CFG <word> <rest>' line as (word, rest), or None on timeout."""
        end = time.time() + timeout
        while time.time() < end:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            m = CFG_RE.search(line)
            if m:
                return m.group(1), m.group(2)
        return None

    def send(self, text):
        data = text.encode("utf-8")
        if b"\n" in data or b"\r" in data:
            raise ValueError("value must not contain a line break")
        self.ser.write(b"CFG " + data + b"\n")
        self.ser.flush()

    def wait_ready(self):
        print(f"Waiting for the firmware (up to {BOOT_WAIT:.0f} s)", end="", flush=True)
        end = time.time() + BOOT_WAIT
        while time.time() < end:
            self.send("PING")
            reply = self.read_cfg(1.0)
            if reply and reply[0] == "PONG":
                print(" ready")
                time.sleep(0.3)
                self.ser.reset_input_buffer()  # drop extra PONGs of repeated pings
                return
            print(".", end="", flush=True)
        print()
        sys.exit("No answer from the device. Is this firmware version flashed, "
                 "and is nothing else using the port?")

    def get(self):
        self.send("GET")
        values = {}
        while True:
            reply = self.read_cfg(REPLY_TIMEOUT)
            if reply is None:
                sys.exit("Timeout while reading the current configuration")
            word, rest = reply
            if word == "VAL":
                key, _, val = rest.partition("=")
                values[key] = val
            elif word == "END":
                return values

    def set(self, key, value):
        """Returns None on success or the device's error text."""
        try:
            self.send(f"SET {key}={value}")
        except ValueError as e:
            return str(e)
        while True:
            reply = self.read_cfg(REPLY_TIMEOUT)
            if reply is None:
                return "no answer from the device"
            word, rest = reply
            if word == "OK" and rest == key:
                return None
            if word == "ERR":
                return rest.split(": ", 1)[-1]

    def save(self):
        """Returns (True, '') once the device confirmed, else (False, reason)."""
        self.send("SAVE")
        while True:
            reply = self.read_cfg(REPLY_TIMEOUT)
            if reply is None:
                return False, "No confirmation from the device"
            if reply[0] == "SAVED":
                return True, ""
            if reply[0] == "ERR":
                return False, reply[1]

    def cancel(self):
        try:
            self.send("CANCEL")
        except Exception:
            pass


def display(key, value):
    if key in ("wifi_pass", "mqtt_pass"):
        return "<set>" if value else "<empty>"
    return value if value else "<empty>"


# ---------------------------------------------------------------------------
# Dialog model (no curses calls, so it can be tested with plain key values)
# ---------------------------------------------------------------------------
try:
    import curses
except ImportError:
    curses = None


class Field:
    def __init__(self, key, label, secret, hint, orig):
        self.key, self.label, self.secret, self.hint = key, label, secret, hint
        self.orig = orig          # value stored in the device ("<set>" marker for passwords)
        self.new = None           # None = unchanged, otherwise the new value ("" = clear)
        self.choices = CHOICES.get(key)

    def shown(self):
        """Text for the value column outside edit mode."""
        if self.new is None:
            return display(self.key, self.orig) if (self.secret or not self.orig) else self.orig
        if self.secret:
            return "*" * len(self.new) if self.new else "<empty>"
        return self.new if self.new else "<empty>"

    def effective(self):
        return self.orig if self.new is None else self.new

    def set_new(self, value):
        """Record a value; equal to the stored one means unchanged."""
        if not self.secret and value == self.orig:
            self.new = None
        elif self.secret and value == "" and not self.orig:
            self.new = None
        else:
            self.new = value


def _key(name, default):
    return getattr(curses, name, default) if curses else default


class Form:
    SAVE, EXIT = "SAVE", "EXIT"

    def __init__(self, current):
        self.fields = [Field(k, l, s, h, current.get(k, "")) for k, l, s, h in FIELDS]
        self.configured = current.get("configured") == "1"
        self.sel = 0              # 0..n-1 fields, n = SAVE, n+1 = EXIT
        self.editing = False
        self.buf = ""
        self.pos = 0
        self.confirm_discard = False
        self.message = ""
        self.message_error = False
        self.error_key = None

    # -- helpers
    @property
    def items(self):
        return len(self.fields) + 2

    def changed(self):
        return [f for f in self.fields if f.new is not None]

    def say(self, text, error=False):
        self.message, self.message_error = text, error

    def field(self):
        return self.fields[self.sel] if self.sel < len(self.fields) else None

    # -- key handling; returns "save", "exit" or None
    def handle_key(self, ch):
        K = _key
        if self.confirm_discard:
            self.confirm_discard = False
            if ch in ("y", "Y"):
                return "exit"
            self.say("")
            return None
        if self.editing:
            return self._edit_key(ch)
        if ch in ("\x1b", "\x03", K("KEY_F10", -10)):
            return self._request_exit()
        if ch == K("KEY_F2", -2):
            return "save"
        if ch in (K("KEY_DOWN", -1), "\t"):
            self.sel = (self.sel + 1) % self.items
        elif ch in (K("KEY_UP", -3), K("KEY_BTAB", -4)):
            self.sel = (self.sel - 1) % self.items
        elif ch in (K("KEY_LEFT", -5), K("KEY_RIGHT", -6)):
            step = -1 if ch == K("KEY_LEFT", -5) else 1
            f = self.field()
            if f is None:
                n = len(self.fields)
                self.sel = n + ((self.sel - n + step) % 2)
            elif f.choices:
                self._cycle(f, step)
        elif ch in ("\n", "\r", K("KEY_ENTER", -7), " "):
            return self._activate()
        elif ch in (K("KEY_DC", -8), K("KEY_BACKSPACE", -9), "\x7f", "\b"):
            self._clear_field()
        elif ch in ("r", "R"):
            f = self.field()
            if f:
                f.new = None
                self.say("")
        return None

    def _request_exit(self):
        if self.changed():
            self.confirm_discard = True
            self.say("Discard unsaved changes? (y/n)", error=True)
            return None
        return "exit"

    def _activate(self):
        n = len(self.fields)
        if self.sel == n:
            return "save"
        if self.sel == n + 1:
            return self._request_exit()
        f = self.field()
        if f.choices:
            self._cycle(f, 1)
        else:
            self.editing = True
            self.buf = "" if f.secret else f.effective()
            self.pos = len(self.buf)
            self.say("")
        return None

    def _cycle(self, f, step):
        cur = f.effective()
        i = f.choices.index(cur) if cur in f.choices else 0
        f.set_new(f.choices[(i + step) % len(f.choices)])
        self.say("")

    def _clear_field(self):
        f = self.field()
        if f is None or f.choices:
            return
        if f.key in REQUIRED:
            self.say(f"{f.label} is required", error=True)
            return
        f.set_new("")
        self.say("")

    def _edit_key(self, ch):
        K = _key
        f = self.field()
        if ch == "\x1b":                                   # cancel editing
            self.editing = False
            return None
        if ch in ("\n", "\r", K("KEY_ENTER", -7)):         # accept
            if f.secret and self.buf == "":
                self.editing = False                       # keep what was there
            elif f.key in REQUIRED and self.buf.strip() == "":
                self.say(f"{f.label} is required", error=True)
            else:
                f.set_new(self.buf)
                self.editing = False
                if self.error_key == f.key:
                    self.error_key = None
                self.say("")
            return None
        if ch == K("KEY_LEFT", -5) or ch == "\x02":
            self.pos = max(0, self.pos - 1)
        elif ch == K("KEY_RIGHT", -6) or ch == "\x06":
            self.pos = min(len(self.buf), self.pos + 1)
        elif ch in (K("KEY_HOME", -11), "\x01"):
            self.pos = 0
        elif ch in (K("KEY_END", -12), "\x05"):
            self.pos = len(self.buf)
        elif ch in (K("KEY_BACKSPACE", -9), "\x7f", "\b"):
            if self.pos:
                self.buf = self.buf[:self.pos - 1] + self.buf[self.pos:]
                self.pos -= 1
        elif ch == K("KEY_DC", -8):
            self.buf = self.buf[:self.pos] + self.buf[self.pos + 1:]
        elif ch == "\x15":                                 # Ctrl+U: clear the line
            self.buf, self.pos = "", 0
        elif isinstance(ch, str) and len(ch) == 1 and ch.isprintable():
            self.buf = self.buf[:self.pos] + ch + self.buf[self.pos:]
            self.pos += 1
        return None

    # -- actions that talk to the device
    def apply(self, dev):
        """Send changed fields and SAVE. Returns True when the device saved and reboots."""
        if not self.changed():
            self.say("Nothing changed")
            return False
        for f in self.fields:
            if f.key in REQUIRED and not f.effective().strip():
                self.sel = self.fields.index(f)
                self.say(f"{f.label} is required", error=True)
                return False
        for f in self.changed():
            err = dev.set(f.key, f.new)
            if err:
                self.sel = self.fields.index(f)
                self.error_key = f.key
                self.say(f"{f.label}: {err}", error=True)
                return False
        self.error_key = None
        ok, why = dev.save()
        if not ok:
            self.say(f"Save failed: {why}", error=True)
        return ok


# ---------------------------------------------------------------------------
# curses drawing
# ---------------------------------------------------------------------------
MIN_H, MIN_W = 19, 60
LABEL_W = 20


def draw(scr, form, port):
    scr.erase()
    h, w = scr.getmaxyx()

    def put(y, x, text, attr=0):
        if 0 <= y < h and 0 <= x < w - 1:
            try:
                scr.addnstr(y, x, text, w - 1 - x, attr)
            except curses.error:
                pass

    if h < MIN_H or w < MIN_W:
        put(0, 0, f"Terminal too small, need {MIN_W}x{MIN_H} (now {w}x{h})")
        scr.refresh()
        return

    red = curses.color_pair(1) if curses.has_colors() else curses.A_BOLD
    green = curses.color_pair(2) if curses.has_colors() else curses.A_BOLD
    dim = curses.A_DIM

    put(0, 1, "the-receiver configuration", curses.A_BOLD)
    put(0, w - len(port) - 2, port, dim)
    put(1, 1, "Device: " + ("configured" if form.configured else "no configuration yet")
        + ("   (unsaved changes: %d)" % len(form.changed()) if form.changed() else ""), dim)
    put(2, 1, "-" * (w - 3), dim)

    vx = 3 + LABEL_W + 2
    vw = max(10, w - vx - 3)
    for i, f in enumerate(form.fields):
        y = 3 + i
        selected = (i == form.sel)
        mark = ">" if selected else " "
        changed = "*" if f.new is not None else " "
        put(y, 1, f"{mark}{changed}{f.label:<{LABEL_W}}", curses.A_BOLD if selected else 0)
        if selected and form.editing:
            text = "*" * len(form.buf) if f.secret else form.buf
            start = max(0, form.pos - vw + 1)
            put(y, vx, "[" + text[start:start + vw].ljust(vw) + "]", curses.A_UNDERLINE)
            cursor = (y, vx + 1 + form.pos - start)
        else:
            attr = curses.A_REVERSE if selected else 0
            if f.key == form.error_key:
                attr |= red
            shown = f.shown()
            if shown.startswith("<") and not selected:
                attr |= dim
            put(y, vx, "[" + shown[:vw].ljust(vw) + "]", attr)

    n = len(form.fields)
    f = form.field()
    hint_y = 3 + n + 1
    if f and f.hint:
        put(hint_y, 3, f.hint, dim)
    if f and f.key in ("wifi_pass", "mqtt_pass"):
        put(hint_y + 1, 3, "Del = clear the stored password", dim)

    by = hint_y + 3
    save_attr = curses.A_REVERSE if form.sel == n else 0
    exit_attr = curses.A_REVERSE if form.sel == n + 1 else 0
    put(by, 3, "[ SAVE ]", save_attr | curses.A_BOLD)
    put(by, 14, "[ EXIT ]", exit_attr | curses.A_BOLD)

    if form.message:
        put(by + 2, 3, form.message, red if form.message_error else green)

    help_text = ("Editing: Enter accept  Esc cancel  Ctrl+U clear" if form.editing else
                 "Up/Down Tab: move  Enter: edit/press  Del: clear  R: revert  F2: save  Esc: exit")
    put(h - 1, 1, help_text, dim)

    try:
        if form.editing:
            curses.curs_set(1)
            scr.move(*cursor)
        else:
            curses.curs_set(0)
    except curses.error:
        pass
    scr.refresh()


def run_dialog(stdscr, dev, current, port):
    form = Form(current)
    try:
        curses.curs_set(0)
    except curses.error:
        pass
    if curses.has_colors():
        curses.start_color()
        try:
            curses.use_default_colors()
            bg = -1
        except curses.error:
            bg = curses.COLOR_BLACK
        curses.init_pair(1, curses.COLOR_RED, bg)
        curses.init_pair(2, curses.COLOR_GREEN, bg)
    stdscr.keypad(True)

    while True:
        draw(stdscr, form, port)
        try:
            ch = stdscr.get_wch()
        except KeyboardInterrupt:
            ch = "\x03"
        except curses.error:
            continue
        if ch == curses.KEY_RESIZE:
            continue
        action = form.handle_key(ch)
        if action == "exit":
            return False
        if action == "save":
            form.say("Saving...")
            draw(stdscr, form, port)
            if form.apply(dev):
                return True


def line_prompt_field(dev, key, label, secret, hint, current):
    """Line mode: ask until the device accepts the value. True if the field was changed."""
    required = key in REQUIRED and not current
    while True:
        shown = display(key, current)
        suffix = f" [{shown}]" if (current or not required) else ""
        extra = f"  ({hint})" if hint and key not in REQUIRED else ""
        text = f"{label}{suffix}{extra}: "
        value = getpass.getpass(text) if secret else input(text)
        if value == "":
            if required:
                print("  required")
                continue
            return False                      # keep the current value
        if value == "-":
            value = ""                        # clear -> default
            if key in REQUIRED:
                print("  required")
                continue
        err = dev.set(key, value)
        if err is None:
            return True
        print(f"  rejected by the device: {err}")


def main():
    ap = argparse.ArgumentParser(description="Configure the-receiver over serial")
    ap.add_argument("port", help="serial port, e.g. COM3 or /dev/cu.usbserial-110")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wifi-ssid")
    ap.add_argument("--wifi-pass")
    ap.add_argument("--mqtt-host")
    ap.add_argument("--mqtt-port")
    ap.add_argument("--mqtt-user")
    ap.add_argument("--mqtt-pass")
    ap.add_argument("--sensor-id", help="hex, 0 = auto")
    ap.add_argument("--channel", help="1, 2 or 3")
    ap.add_argument("--ntp")
    ap.add_argument("--gmt", help="hours, empty string = Warsaw with DST")
    ap.add_argument("-y", "--yes", action="store_true", help="save without the final question")
    ap.add_argument("--text", action="store_true", help="plain line prompts instead of the dialog")
    ap.add_argument("--no-watch", action="store_true", help="do not show the log after the reboot")
    args = ap.parse_args()

    given = {key: getattr(args, key) for key, *_ in FIELDS if getattr(args, key) is not None}

    use_dialog = not given and not args.text
    if use_dialog and curses is None:
        print("The dialog needs the curses module (Windows: pip install windows-curses); "
              "using line prompts.")
        use_dialog = False
    if use_dialog and not (sys.stdin.isatty() and sys.stdout.isatty()):
        print("Not a terminal; using line prompts.")
        use_dialog = False

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {args.port}: {e}")

    with ser:
        dev = Device(ser)
        dev.wait_ready()
        current = dev.get()
        configured = current.get("configured") == "1"
        print("Device is configured." if configured else "Device has no configuration yet.")

        saved = False
        if use_dialog:
            os.environ.setdefault("ESCDELAY", "25")   # make a lone Esc react quickly
            try:
                saved = curses.wrapper(run_dialog, dev, current, args.port)
            except KeyboardInterrupt:
                saved = False
            if not saved:
                dev.cancel()
                print("Exited without saving.")
                return
        else:
            changed = []
            try:
                if given:
                    for key, *_ in FIELDS:
                        if key in given:
                            err = dev.set(key, given[key])
                            if err:
                                dev.cancel()
                                sys.exit(f"{key}: rejected by the device: {err}")
                            changed.append(key)
                else:
                    print("Enter keeps the current value, '-' clears it.\n")
                    for key, label, secret, hint in FIELDS:
                        if line_prompt_field(dev, key, label, secret, hint, current.get(key, "")):
                            changed.append(key)
            except (KeyboardInterrupt, EOFError):
                dev.cancel()
                sys.exit("\nCancelled, nothing was saved.")

            if not changed:
                print("Nothing changed.")
                dev.cancel()
                return

            final = dev.get()  # staged values as the device sees them
            print("\nNew configuration:")
            for key, label, *_ in FIELDS:
                mark = "*" if key in changed else " "
                print(f" {mark} {label:<20} {display(key, final.get(key, ''))}")
            if not args.yes:
                try:
                    answer = input("Save to EEPROM and reboot? [Y/n] ").strip().lower()
                except (KeyboardInterrupt, EOFError):
                    answer = "n"
                if answer not in ("", "y", "yes"):
                    dev.cancel()
                    sys.exit("Cancelled, nothing was saved.")
            saved, why = dev.save()
            if not saved:
                sys.exit(f"Save failed: {why}")

        print("Saved. The device is rebooting.")
        if not args.no_watch:
            print(f"--- device log, {WATCH_SECONDS:.0f} s ---")
            end = time.time() + WATCH_SECONDS
            while time.time() < end:
                raw = ser.readline()
                if raw:
                    print(raw.decode("utf-8", errors="replace").rstrip("\r\n"))


if __name__ == "__main__":
    main()
