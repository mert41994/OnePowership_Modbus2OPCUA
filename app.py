#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import re
import struct
import threading
import time
import json
import os
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from pymodbus.client import ModbusTcpClient
from opcua import Server, ua

# ---------------- Kalıcı ayar yolu ----------------
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "mod2opc_config.json")

# ---------------- Yardımcılar ----------------

DATA_TYPES = [
    "BOOL", "UINT16", "INT16",
    "UINT32_LE", "UINT32_BE",
    "INT32_LE", "INT32_BE",
    "FLOAT32_LE", "FLOAT32_BE",
]

def decode_registers(registers, dtype):
    if dtype == "UINT16":
        return registers[0] & 0xFFFF
    if dtype == "INT16":
        return struct.unpack("!h", struct.pack("!H", registers[0] & 0xFFFF))[0]
    if dtype in ("UINT32_LE", "INT32_LE", "FLOAT32_LE"):
        raw = struct.pack("<HH", (registers[0] & 0xFFFF), (registers[1] & 0xFFFF))
        if dtype == "UINT32_LE": return struct.unpack("<I", raw)[0]
        if dtype == "INT32_LE":  return struct.unpack("<i", raw)[0]
        if dtype == "FLOAT32_LE":return struct.unpack("<f", raw)[0]
    if dtype in ("UINT32_BE", "INT32_BE", "FLOAT32_BE"):
        raw = struct.pack(">HH", (registers[0] & 0xFFFF), (registers[1] & 0xFFFF))
        if dtype == "UINT32_BE": return struct.unpack(">I", raw)[0]
        if dtype == "INT32_BE":  return struct.unpack(">i", raw)[0]
        if dtype == "FLOAT32_BE":return struct.unpack(">f", raw)[0]
    raise ValueError(f"Desteklenmeyen dtype: {dtype}")

def required_quantity(dtype, fn):
    if fn in ("Coils", "Discrete Inputs"): return 1
    if dtype in ("BOOL", "UINT16", "INT16"): return 1
    return 2

# ---------------- Tag modeli ----------------

class Tag:
    def __init__(self, name, function, address, dtype, unit_id=1):
        self.name = name.strip()
        self.function = function.strip()
        self.address = int(address)
        self.dtype = dtype.strip()
        self.unit_id = int(unit_id)
        self.ua_node = None
    def __repr__(self):
        return f"<Tag {self.name} {self.function}@{self.address} {self.dtype} u{self.unit_id}>"

    def to_dict(self):
        return {
            "name": self.name,
            "function": self.function,
            "address": self.address,
            "dtype": self.dtype,
            "unit_id": self.unit_id,
        }

    @staticmethod
    def from_dict(d):
        return Tag(
            name=d["name"],
            function=d["function"],
            address=int(d["address"]),
            dtype=d["dtype"],
            unit_id=int(d.get("unit_id", 1)),
        )

# ---------------- Adres tabanı ----------------

BASE_1 = {
    "Coils": 1,
    "Discrete Inputs": 10001,
    "Input Registers": 30001,
    "Holding Registers": 40001
}

def to_zero_based_addr(addr_input, function, is_ui_one_based):
    """
    Akıllı dönüşüm:
    - 40020 gibi 1-based absolute değerler (>= base) -> addr - base
    - 0-based offset (küçük değerler) -> değişmeden
    - UI 1-based seçiliyse ve küçük değer girilmişse (örn: 1) -> addr - 1
    """
    base = BASE_1.get(function)
    if base is None:
        raise ValueError(f"Bilinmeyen function: {function}")

    # Absolute 1-based algılama
    if addr_input >= base:
        off = addr_input - base
        if off < 0:
            raise ValueError(f"Adres {addr_input} ({function}) tabandan küçük (>= {base} olmalı).")
        return off

    # Absolute değilse, UI tercihine göre yorumla
    if is_ui_one_based:
        # İnsan gözüyle 1,2,3... girildiyse 1->0, 2->1
        return max(addr_input - 1, 0)
    else:
        # 0-based offset olarak kullan
        return addr_input

# ---------------- Modbus okuma ----------------

class ModbusReader:
    def __init__(self, host, port, ui_one_based=False, timeout=3.0):
        self.host, self.port = host, int(port)
        self.ui_one_based = ui_one_based
        self.timeout = float(timeout)
        self.client = None
        self.lock = threading.Lock()

    def connect(self):
        with self.lock:
            self.client = ModbusTcpClient(self.host, port=self.port, timeout=self.timeout)
            return self.client.connect()

    def close(self):
        with self.lock:
            if self.client:
                self.client.close()
                self.client = None

    def read_tag(self, tag: Tag):
        if self.client is None:
            raise RuntimeError("Modbus client bağlı değil")

        addr = to_zero_based_addr(tag.address, tag.function, self.ui_one_based)
        qty = required_quantity(tag.dtype, tag.function)
        fn = tag.function

        if fn == "Coils":
            rr = self.client.read_coils(addr, device_id=tag.unit_id)
            if rr.isError(): raise RuntimeError(f"Read Coils hata: {rr}")
            return bool(rr.bits[0])

        if fn == "Discrete Inputs":
            rr = self.client.read_discrete_inputs(addr, device_id=tag.unit_id)
            if rr.isError(): raise RuntimeError(f"Read DI hata: {rr}")
            return bool(rr.bits[0])

        if fn == "Holding Registers":
            rr = self.client.read_holding_registers(addr, device_id=tag.unit_id)
            if rr.isError(): raise RuntimeError(f"Read HR hata: {rr}")
            regs = rr.registers
            if not regs or len(regs) < qty:
                raise RuntimeError(f"Read HR eksik veri: beklenen {qty}, gelen {len(regs) if regs else 0}")
            return bool(regs[0] & 1) if tag.dtype == "BOOL" else decode_registers(regs, tag.dtype)

        if fn == "Input Registers":
            rr = self.client.read_input_registers(addr, device_id=tag.unit_id)
            if rr.isError(): raise RuntimeError(f"Read IR hata: {rr}")
            regs = rr.registers
            if not regs or len(regs) < qty:
                raise RuntimeError(f"Read IR eksik veri: beklenen {qty}, gelen {len(regs) if regs else 0}")
            return bool(regs[0] & 1) if tag.dtype == "BOOL" else decode_registers(regs, tag.dtype)

        raise ValueError(f"Bilinmeyen function: {fn}")

# ---------------- OPC UA yayın ----------------

class OpcUaPublisher:
    def __init__(self, endpoint, namespace_uri, root1, root2):
        self.endpoint = endpoint
        self.namespace_uri = namespace_uri
        self.root1 = root1.strip() or "RootA"
        self.root2 = root2.strip() or "RootB"
        self.server = None
        self.ns_idx = None
        self.root_node = None

    def start(self):
        self.server = Server()
        self.server.set_endpoint(self.endpoint)
        self.server.set_server_name("Modbus2OPC UA Bridge")
        self.ns_idx = self.server.register_namespace(self.namespace_uri)
        objects = self.server.get_objects_node()
        lvl1 = objects.add_folder(self.ns_idx, self._clean(self.root1))
        lvl2 = lvl1.add_folder(self.ns_idx, self._clean(self.root2))
        self.root_node = lvl2
        self.server.start()

    def stop(self):
        if self.server:
            try: self.server.stop()
            except Exception: pass
            self.server = None

    def add_variable(self, name, initial_value=None):
        node = self.root_node.add_variable(self.ns_idx, self._clean(name), initial_value)
        return node

    def write(self, node, value):
        node.set_value(ua.Variant(value, self._guess_variant_type(value)))

    def _clean(self, name):
        return name.replace("/", "_").replace(".", "_").strip()

    def _guess_variant_type(self, value):
        if isinstance(value, bool): return ua.VariantType.Boolean
        if isinstance(value, int): return ua.VariantType.Int64
        if isinstance(value, float): return ua.VariantType.Double
        return ua.VariantType.String

# ---------------- Köprü ----------------

class Bridge:
    def __init__(self, reader, publisher, tags, poll_ms, log_cb):
        self.reader, self.publisher = reader, publisher
        self.tags = list(tags)
        self.poll_ms = int(poll_ms)
        self.log_cb = log_cb or (lambda m: None)
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if not self.reader.connect():
            raise RuntimeError("Modbus sunucuya bağlanılamadı")
        self.publisher.start()
        for tag in self.tags:
            tag.ua_node = self.publisher.add_variable(tag.name, 0)
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.reader.close()
        self.publisher.stop()

    def _loop(self):
        while not self._stop.is_set():
            t0 = time.time()
            for tag in self.tags:
                try:
                    val = self.reader.read_tag(tag)
                    if tag.ua_node:
                        self.publisher.write(tag.ua_node, val)
                    self.log_cb(f"OK [{tag.name}] = {val}")
                except Exception as e:
                    self.log_cb(f"ERR [{tag.name}] {e}")
            elapsed = (time.time() - t0) * 1000.0
            time.sleep(max(0, self.poll_ms - elapsed) / 1000.0)

# ---------------- Kepware CSV import ----------------

def parse_kepware_address(addr_str):
    s = addr_str.strip().upper().replace(" ", "")
    # Örnekler: HR40020, IR30005, DI10001, C1
    if s.startswith("HR"): return "Holding Registers", int(re.sub(r"\D","", s[2:]))
    if s.startswith("IR"): return "Input Registers", int(re.sub(r"\D","", s[2:]))
    if s.startswith("DI"): return "Discrete Inputs", int(re.sub(r"\D","", s[2:]))
    if s.startswith("C"):  return "Coils", int(re.sub(r"\D","", s[1:]))

    # 4X/3X/1X/0X + sayı
    m = re.match(r"^([0-4])X?(\d+)$", s)
    if m:
        lead, num = m.group(1), int(m.group(2))
        func = {"4":"Holding Registers","3":"Input Registers","1":"Discrete Inputs","0":"Coils"}[lead]
        return func, num

    # Sadece sayı (40020, 30005, 10001 ya da offset gibi)
    if s.isdigit():
        num = int(s)
        if num >= 40001: return "Holding Registers", num
        if num >= 30001: return "Input Registers", num
        if num >= 10001: return "Discrete Inputs", num
        # 0/1 taban belirsiz küçük değer: Coils olarak kabul etmek daha güvenli değil;
        # adreslemeyi tag eklerken fonksiyonla birlikte belirtiyoruz. Varsayılan Coils:
        return "Coils", num

    raise ValueError(f"Adres çözümlenemedi: {addr_str}")

KEPWARE_DTYPE_MAP = {
    "BOOL":"BOOL", "BOOLEAN":"BOOL",
    "INT":"INT16", "SHORT":"INT16",
    "WORD":"UINT16", "UINT":"UINT16", "UINT16":"UINT16",
    "DINT":"INT32_LE", "INT32":"INT32_LE", "DWORD":"UINT32_LE", "UINT32":"UINT32_LE",
    "REAL":"FLOAT32_LE", "FLOAT":"FLOAT32_LE", "FLOAT32":"FLOAT32_LE",
}



# ---------------- Tkinter UI ----------------

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("OnePowership Modbus → OPC UA Köprüsü")
        self.tags, self.bridge = [], None

        frm = ttk.Frame(self)
        frm.pack(fill="x", padx=8, pady=8)

        messagebox.showinfo("OnePowership Modbus → OPC UA Köprüsü", "Bu uygulama OnePowership projesi için geliştirilmiştir.\n2025\nOnePowership Software Development Team")

        mb = ttk.LabelFrame(frm, text="Modbus TCP")
        mb.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self.mb_host = self._labeled_entry(mb, "Host:", "127.0.0.1", 0)
        self.mb_port = self._labeled_entry(mb, "Port:", "502", 1)
        self.mb_unit = self._labeled_entry(mb, "Unit ID:", "1", 2)
        self.addr_base = tk.StringVar(value="0-based")  # Görsel tercih
        ttk.Radiobutton(mb, text="0-based (offset)", variable=self.addr_base, value="0-based").grid(row=3, column=0, sticky="w")
        ttk.Radiobutton(mb, text="1-based (40001..)", variable=self.addr_base, value="1-based").grid(row=3, column=1, sticky="w")
        self.poll = self._labeled_entry(mb, "Poll (ms):", "1000", 4)
        ttk.Button(self, text="Seçili Tag'i Düzenle", command=self.edit_tag).pack(padx=8, pady=2, anchor="e")

        ua_frm = ttk.LabelFrame(frm, text="OPC UA Server")
        ua_frm.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        self.ua_endpoint = self._labeled_entry(ua_frm, "Endpoint:", "opc.tcp://0.0.0.0:4840", 0)
        self.ua_ns = self._labeled_entry(ua_frm, "Namespace URI:", "urn:mod2opc:bridge", 1)
        self.ua_root1 = self._labeled_entry(ua_frm, "manuelDeger:", "PlantA", 2)
        self.ua_root2 = self._labeled_entry(ua_frm, "manuelDeger2:", "Line1", 3)

        tg = ttk.LabelFrame(self, text="Tag Ekle")
        tg.pack(fill="x", padx=8, pady=4)
        self.tag_name = self._labeled_entry(tg, "Tag Adı:", "HR_40001", 0)
        ttk.Label(tg, text="Fonksiyon:").grid(row=0, column=2, padx=4, pady=2, sticky="e")
        self.tag_func = ttk.Combobox(tg, values=["Holding Registers","Input Registers","Coils","Discrete Inputs"], state="readonly", width=20)
        self.tag_func.current(0)
        self.tag_func.grid(row=0, column=3, padx=4, pady=2)
        self.tag_addr = self._labeled_entry(tg, "Adres:", "40001", 1)
        ttk.Label(tg, text="DataType:").grid(row=1, column=2, padx=4, pady=2, sticky="e")
        self.tag_dtype = ttk.Combobox(tg, values=DATA_TYPES, state="readonly", width=20)
        self.tag_dtype.current(1)  # UINT16
        self.tag_dtype.grid(row=1, column=3, padx=4, pady=2)
        ttk.Button(tg, text="Ekle", command=self.add_tag).grid(row=0, column=4, rowspan=2, padx=8, pady=2, sticky="ns")
        ttk.Button(tg, text="Kepware CSV Import", command=self.import_kepware_csv).grid(row=0, column=5, rowspan=2, padx=8, pady=2, sticky="ns")
        ttk.Button(tg, text="Test Modbus", command=self.test_modbus).grid(row=0, column=6, rowspan=2, padx=8, pady=2, sticky="ns")

        lstfrm = ttk.LabelFrame(self, text="Tag Listesi")
        lstfrm.pack(fill="both", expand=True, padx=8, pady=4)
        columns = ("name","function","address","dtype")
        self.tree = ttk.Treeview(lstfrm, columns=columns, show="headings", height=8)
        for c in columns:
            self.tree.heading(c, text=c)
            self.tree.column(c, minwidth=80, width=140, stretch=True)
        self.tree.pack(side="left", fill="both", expand=True)
        scr = ttk.Scrollbar(lstfrm, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscroll=scr.set)
        scr.pack(side="right", fill="y")
        ttk.Button(self, text="Seçili Tag'i Sil", command=self.del_tag).pack(padx=8, pady=2, anchor="e")

        ctl = ttk.Frame(self)
        ctl.pack(fill="x", padx=8, pady=6)
        self.start_btn = ttk.Button(ctl, text="Start", command=self.start_bridge)
        self.stop_btn = ttk.Button(ctl, text="Stop", command=self.stop_bridge, state="disabled")
        self.start_btn.pack(side="left", padx=4)
        self.stop_btn.pack(side="left", padx=4)

        logfrm = ttk.LabelFrame(self, text="Log")
        logfrm.pack(fill="both", expand=True, padx=8, pady=6)
        self.log = tk.Text(logfrm, height=12)
        self.log.pack(fill="both", expand=True)

        # Konfigürasyonu açılışta yükle
        self.load_config()

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _labeled_entry(self, parent, label, default, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, padx=4, pady=2, sticky="e")
        ent = ttk.Entry(parent, width=24)
        ent.insert(0, str(default))
        ent.grid(row=row, column=1, padx=4, pady=2, sticky="w")
        return ent

    def edit_tag(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showwarning("Uyarı", "Düzenlemek için bir tag seçin.")
            return
        idx = self.tree.index(sel[0])
        t = self.tags[idx]

        # Mevcut değerleri UI alanlarına doldur
        self.tag_name.delete(0, "end")
        self.tag_name.insert(0, t.name)
        self.tag_func.set(t.function)
        self.tag_addr.delete(0, "end")
        self.tag_addr.insert(0, str(t.address))
        self.tag_dtype.set(t.dtype)

        # Kaydetmek için geçici olarak mevcut index’i sakla
        self._editing_tag_index = idx

        # “Ekle” butonunu “Kaydet”e çevir
        for child in self.tag_name.master.winfo_children():
            if isinstance(child, ttk.Button) and child["text"] == "Ekle":
                child.config(text="Kaydet", command=self.save_edited_tag)

    # --------- Config (save/load) ---------

    def get_config(self):
        return {
            "modbus": {
                "host": self.mb_host.get().strip(),
                "port": self.mb_port.get().strip(),
                "unit": self.mb_unit.get().strip(),
                "addr_base": self.addr_base.get(),
                "poll_ms": self.poll.get().strip(),
            },
            "opcua": {
                "endpoint": self.ua_endpoint.get().strip(),
                "namespace": self.ua_ns.get().strip(),
                "root1": self.ua_root1.get().strip(),
                "root2": self.ua_root2.get().strip(),
            },
            "tags": [t.to_dict() for t in self.tags],
        }

    def apply_config(self, cfg):
        try:
            mb = cfg.get("modbus", {})
            self.mb_host.delete(0, "end"); self.mb_host.insert(0, mb.get("host", "127.0.0.1"))
            self.mb_port.delete(0, "end"); self.mb_port.insert(0, mb.get("port", "502"))
            self.mb_unit.delete(0, "end"); self.mb_unit.insert(0, mb.get("unit", "1"))
            self.addr_base.set(mb.get("addr_base", "0-based"))
            self.poll.delete(0, "end"); self.poll.insert(0, mb.get("poll_ms", "1000"))

            ua = cfg.get("opcua", {})
            self.ua_endpoint.delete(0, "end"); self.ua_endpoint.insert(0, ua.get("endpoint", "opc.tcp://0.0.0.0:4840"))
            self.ua_ns.delete(0, "end"); self.ua_ns.insert(0, ua.get("namespace", "urn:mod2opc:bridge"))
            self.ua_root1.delete(0, "end"); self.ua_root1.insert(0, ua.get("root1", "PlantA"))
            self.ua_root2.delete(0, "end"); self.ua_root2.insert(0, ua.get("root2", "Line1"))

            # Tag listesi
            self.tags = []
            for i in self.tree.get_children():
                self.tree.delete(i)
            for td in cfg.get("tags", []):
                t = Tag.from_dict(td)
                self.tags.append(t)
                self.tree.insert("", "end", values=(t.name, t.function, t.address, t.dtype))
        except Exception as e:
            self._log(f"Config apply hatası: {e}")

    def save_config(self):
        try:
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(self.get_config(), f, ensure_ascii=False, indent=2)
            self._log("Ayarlar kaydedildi.")
        except Exception as e:
            self._log(f"Ayarlar kaydedilemedi: {e}")

    def load_config(self):
        if not os.path.exists(CONFIG_PATH):
            return
        try:
            with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.apply_config(cfg)
            self._log("Ayarlar yüklendi.")
        except Exception as e:
            self._log(f"Ayarlar yüklenemedi: {e}")

    # --------- UI Aksiyonları ---------

    def add_tag(self):
        try:
            name = self.tag_name.get().strip()
            function = self.tag_func.get().strip()
            addr_str = self.tag_addr.get().strip()
            dtype = self.tag_dtype.get().strip()
            if not name: raise ValueError("Tag adı boş olamaz")
            if not addr_str.isdigit(): raise ValueError("Adres bir tam sayı olmalı")
            addr = int(addr_str)
            if any(x.name == name for x in self.tags):
                raise ValueError("Aynı isimde bir tag zaten var")
            t = Tag(name=name, function=function, address=addr, dtype=dtype, unit_id=int(self.mb_unit.get()))
            self.tags.append(t)
            self.tree.insert("", "end", values=(t.name, t.function, t.address, t.dtype))
            self.save_config()
        except Exception as e:
            messagebox.showerror("Hata", str(e))

    def del_tag(self):
        sel = self.tree.selection()
        if not sel: return
        idx = self.tree.index(sel[0])
        self.tree.delete(sel[0])
        del self.tags[idx]
        self.save_config()

    def import_kepware_csv(self):
        path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if not path: return
        loaded = 0
        try:
            with open(path, newline='', encoding="utf-8-sig") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    name = row.get("Tag Name","").strip()
                    addr_s = row.get("Address","").strip()
                    dtype_s = row.get("Data Type","").strip().upper()
                    if not name or not addr_s: continue
                    func, addr = parse_kepware_address(addr_s)
                    dtype = KEPWARE_DTYPE_MAP.get(dtype_s, "UINT16")
                    t = Tag(name=name, function=func, address=addr, dtype=dtype, unit_id=int(self.mb_unit.get()))
                    if any(x.name == t.name for x in self.tags):
                        continue
                    self.tags.append(t); loaded += 1
                    self.tree.insert("", "end", values=(t.name, t.function, t.address, t.dtype))
            self._log(f"CSV import tamam: {loaded} tag eklendi.")
            self.save_config()
        except Exception as e:
            messagebox.showerror("CSV Hatası", str(e))

    def test_modbus(self):
        try:
            host = self.mb_host.get().strip()
            port = int(self.mb_port.get().strip())
            unit = int(self.mb_unit.get().strip())
            ui_one_based = (self.addr_base.get() == "1-based")
            reader = ModbusReader(host, port, ui_one_based=ui_one_based)
            if not reader.connect():
                raise RuntimeError("Modbus sunucuya bağlanılamadı")
            try:
                if self.tags:
                    val = reader.read_tag(self.tags[0])
                    self._log(f"Test OK [{self.tags[0].name}] = {val}")
                else:
                    # HR 40020/19 testi; UI moduna göre iki yazımdan biri
                    dummy_addr = 40001 if ui_one_based else 0
                    dummy = Tag("TEST_HR", "Holding Registers", dummy_addr, "UINT16", unit)
                    val = reader.read_tag(dummy)
                    self._log(f"Test OK [HR] = {val}")
            finally:
                reader.close()
        except Exception as e:
            self._log(f"Test ERR: {e}")
            messagebox.showerror("Test Hatası", str(e))

    def start_bridge(self):
        if self.bridge:
            messagebox.showwarning("Uyarı", "Bridge zaten çalışıyor")
            return
        try:
            if not self.tags:
                raise ValueError("En az bir tag ekleyin.")
            host = self.mb_host.get().strip()
            port = int(self.mb_port.get().strip())
            unit = int(self.mb_unit.get().strip())
            ui_one_based = (self.addr_base.get() == "1-based")
            poll_ms = int(self.poll.get().strip())

            # Tag'lerin Unit ID'sini UI'ya eşitle
            for t in self.tags: t.unit_id = unit

            reader = ModbusReader(host, port, ui_one_based=ui_one_based)
            publisher = OpcUaPublisher(
                self.ua_endpoint.get().strip(),
                self.ua_ns.get().strip(),
                self.ua_root1.get().strip(),
                self.ua_root2.get().strip()
            )
            self.bridge = Bridge(reader, publisher, self.tags, poll_ms, self._log)
            self.bridge.start()
            self._log(f"Bridge başladı: Modbus {host}:{port} u{unit} | OPC UA {self.ua_endpoint.get().strip()}")
            self.start_btn.config(state="disabled")
            self.stop_btn.config(state="normal")
            self.save_config()
        except Exception as e:
            self._log(f"ERR start: {e}")
            self.bridge = None
            messagebox.showerror("Başlatılamadı", str(e))

    def stop_bridge(self):
        if not self.bridge: return
        try:
            self.bridge.stop()
            self._log("Bridge durdu.")
        except Exception as e:
            self._log(f"ERR stop: {e}")
        finally:
            self.bridge = None
            self.start_btn.config(state="normal")
            self.stop_btn.config(state="disabled")
            self.save_config()

    def _log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.log.insert("end", f"{ts} | {msg}\n")
        self.log.see("end")

    def on_close(self):
        try:
            if self.bridge:
                self.bridge.stop()
            self.save_config()
        finally:
            self.destroy()

if __name__ == "__main__":
    App().mainloop()
