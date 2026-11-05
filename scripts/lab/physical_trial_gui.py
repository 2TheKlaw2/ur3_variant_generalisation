#!/usr/bin/env python3
"""Physical trial launcher for the UR3e capstone (V1-V5, L0/L1/L1D/L2/CAD), with run review and operator notes.

  python3 scripts/lab/physical_trial_gui.py          (Ubuntu: sudo apt install python3-tk if tkinter is missing)

SAFETY: this window never sends ROS commands itself.
  1. "Preflight" runs run_full_cycle_physical.py WITHOUT --execute (offline, no ROS, no motion).
  2. "Run on robot" is enabled only after that exact configuration passed preflight. It opens a terminal that
     sources ROS, sets ROS_DOMAIN_ID=0, optionally resets the MoveIt scene (planning_scene_tools.py --reset
     --physical) and starts the SAME runner with --execute. Every prompt (RUN_<STAGE>, or with --auto only the
     cycle start and RUN_INSERT) is answered in that terminal, all runner checks apply, Ctrl+C there cancels
     the active stage. The pendant stop is the emergency stop.
"""
import csv
import json
import queue
import shlex
import shutil
import subprocess
import threading
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk


def find_repo():
    for p in (Path(__file__).resolve().parents[2], Path.cwd(), Path.home() / "ros2_ws/src/ur3_variant_generalisation"):
        if (p / "scripts/phase11_physical/run_full_cycle_physical.py").is_file():
            return p
    raise SystemExit("Repo not found: keep this file in <repo>/scripts/lab/.")


ROOT = find_repo()
RUNNER = "scripts/phase11_physical/run_full_cycle_physical.py"
SCENE = "scripts/phase6_pickup_sim/planning_scene_tools.py"
PLANS_DEFAULT = "results/simulation/stage10_depthfix"
RUNS_DIR = ROOT / "results/physical/full_cycle"
NOTES_FILE = ROOT / "results/physical/operator_trial_notes.csv"
# what the runner allows (assembles and seats in simulation, session 20261103_1537)
ALLOWED = {1: ("L2", "CAD", "L1", "L1D", "L0"), 2: ("L2", "CAD", "L1"), 3: ("L2", "CAD", "L1", "L1D"),
           4: ("L2", "CAD"), 5: ("L2", "CAD")}
LEVEL_INFO = {"L2": "CAD corridor + demo timing (the method)", "CAD": "CAD-only baseline", "L1": "classic DMP scaling",
              "L1D": "Victor's CDMP, goal only", "L0": "demo replay"}
ORDER = ["HOME_START", "OPEN", "REACH", "ALIGN_GRIP", "GRIP", "LIFT", "TRANSPORT", "ALIGN_INSERT", "INSERT",
         "ROTATE", "RELEASE", "RETREAT", "HOME"]
NOTE_FIELDS = ["time", "run", "variant", "level", "seated", "locked", "contact", "plate_lift", "assistance",
               "part_top_mm", "notes"]
PREFLIGHT_TIMEOUT_S = 600          # a level's first preflight after a registration change recomputes its plan

# palette
INK, INK2, MUTED = "#1d2733", "#4a5868", "#7b8794"
BG, CARD, LINE = "#f3f5f8", "#ffffff", "#dde3ea"
NAVY, ACCENT, ACCENT_D = "#14213d", "#2a78d6", "#1c5cab"
GO, GO_D, WARN, BAD, OK = "#e2622b", "#c24f1c", "#b7791f", "#c53030", "#2f855a"
CONSOLE_BG, CONSOLE_FG = "#0f1722", "#d7e3ee"
FONT = "DejaVu Sans"
MONO = "DejaVu Sans Mono"


class Card(tk.Frame):
    def __init__(self, parent, title, subtitle=None, **kw):
        super().__init__(parent, bg=CARD, highlightbackground=LINE, highlightthickness=1, **kw)
        head = tk.Frame(self, bg=CARD)
        head.pack(fill="x", padx=18, pady=(14, 4))
        tk.Label(head, text=title, bg=CARD, fg=INK, font=(FONT, 12, "bold")).pack(anchor="w")
        if subtitle:
            tk.Label(head, text=subtitle, bg=CARD, fg=MUTED, font=(FONT, 9)).pack(anchor="w")
        self.body = tk.Frame(self, bg=CARD)
        self.body.pack(fill="both", expand=True, padx=18, pady=(6, 16))


class Segmented(tk.Frame):
    """Row of toggle buttons bound to a StringVar; options can be disabled."""

    def __init__(self, parent, var, options, labels=None):
        super().__init__(parent, bg=CARD)
        self.var, self.buttons = var, {}
        for o in options:
            b = tk.Label(self, text=(labels or {}).get(o, o), font=(FONT, 10, "bold"), padx=14, pady=6,
                         cursor="hand2", bd=0, highlightthickness=1)
            b.pack(side="left", padx=(0, 6))
            b.bind("<Button-1>", lambda _e, o=o: self._pick(o))
            self.buttons[o] = b
        self.enabled = set(options)
        var.trace_add("write", lambda *_: self.paint())
        self.paint()

    def _pick(self, o):
        if o in self.enabled:
            self.var.set(o)

    def set_enabled(self, opts):
        self.enabled = set(opts)
        self.paint()

    def paint(self):
        for o, b in self.buttons.items():
            if o not in self.enabled:
                b.configure(bg="#eef1f4", fg="#b5bec8", highlightbackground="#e3e8ee", cursor="arrow")
            elif self.var.get() == o:
                b.configure(bg=ACCENT, fg="white", highlightbackground=ACCENT, cursor="hand2")
            else:
                b.configure(bg=CARD, fg=INK2, highlightbackground=LINE, cursor="hand2")


class Btn(tk.Label):
    def __init__(self, parent, text, cmd, color, dark):
        super().__init__(parent, text=text, bg=color, fg="white", font=(FONT, 11, "bold"), padx=18, pady=9,
                         cursor="hand2")
        self.cmd, self.color, self.dark, self.on = cmd, color, dark, True
        self.bind("<Button-1>", lambda _e: self.on and self.cmd())
        self.bind("<Enter>", lambda _e: self.on and self.configure(bg=self.dark))
        self.bind("<Leave>", lambda _e: self.on and self.configure(bg=self.color))

    def enable(self, on):
        self.on = on
        self.configure(bg=self.color if on else "#c9d1da", fg="white", cursor="hand2" if on else "arrow")


class TrialGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UR3e physical trials")
        self.geometry("1180x800")
        self.minsize(1040, 700)
        self.configure(bg=BG)
        self._style()
        self.variant, self.level = tk.StringVar(value="1"), tk.StringVar(value="L2")
        self.temporal, self.speed = tk.StringVar(value="fixed"), tk.StringVar(value="0.30")
        self.from_stage, self.until = tk.StringVar(value="HOME_START"), tk.StringVar(value="HOME")
        self.plans = tk.StringVar(value=PLANS_DEFAULT)
        self.auto, self.reset_scene = tk.BooleanVar(value=True), tk.BooleanVar(value=True)
        self.passed_cfg, self.busy, self.q = None, False, queue.Queue()
        self._header()
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=20, pady=(14, 18))
        self.nb = nb
        self.tab_run, self.tab_runs, self.tab_notes = (tk.Frame(nb, bg=BG) for _ in range(3))
        nb.add(self.tab_run, text="  Run a trial  ")
        nb.add(self.tab_runs, text="  Previous runs  ")
        nb.add(self.tab_notes, text="  Observations  ")
        self._run_tab()
        self._runs_tab()
        self._notes_tab()
        for v in (self.variant, self.level, self.temporal, self.speed, self.from_stage, self.until, self.plans,
                  self.auto, self.reset_scene):
            v.trace_add("write", self._changed)
        self.variant.trace_add("write", self._variant_changed)
        self._changed()
        self.refresh_runs()

    # ---------- look ----------
    def _style(self):
        st = ttk.Style(self)
        st.theme_use("clam")
        st.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        st.configure("TNotebook.Tab", font=(FONT, 10, "bold"), padding=(14, 8), background="#e4e9ef",
                     foreground=INK2, borderwidth=0)
        st.map("TNotebook.Tab", background=[("selected", CARD)], foreground=[("selected", ACCENT)])
        st.configure("TCombobox", padding=5, arrowsize=14)
        st.configure("Treeview", font=(FONT, 10), rowheight=28, background=CARD, fieldbackground=CARD,
                     foreground=INK, borderwidth=0)
        st.configure("Treeview.Heading", font=(FONT, 9, "bold"), background="#eef2f6", foreground=INK2,
                     relief="flat", padding=(6, 6))
        st.map("Treeview", background=[("selected", "#dbe8f8")], foreground=[("selected", INK)])
        st.configure("TCheckbutton", background=CARD, foreground=INK, font=(FONT, 10))
        st.map("TCheckbutton", background=[("active", CARD)])

    def _header(self):
        h = tk.Frame(self, bg=NAVY)
        h.pack(fill="x")
        inner = tk.Frame(h, bg=NAVY)
        inner.pack(fill="x", padx=24, pady=14)
        tk.Label(inner, text="UR3e  ·  variant assembly trials", bg=NAVY, fg="white",
                 font=(FONT, 17, "bold")).pack(side="left")
        tk.Label(inner, text=f"  {ROOT.name}  ·  staged runner, operator-confirmed", bg=NAVY, fg="#9fb3c8",
                 font=(FONT, 10)).pack(side="left", pady=(6, 0))
        self.pill = tk.Label(inner, text="", font=(FONT, 10, "bold"), padx=12, pady=4)
        self.pill.pack(side="right")

    def _set_pill(self, text, color):
        self.pill.configure(text=text, bg=color, fg="white")

    # ---------- run tab ----------
    def _field(self, parent, label, widget_fn, hint=None):
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=(0, 12))
        tk.Label(row, text=label.upper(), bg=CARD, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 4))
        w = widget_fn(row)
        w.pack(anchor="w")
        if hint is not None:
            hint.master = row
        return w

    def _run_tab(self):
        f = self.tab_run
        f.columnconfigure(0, weight=0, minsize=470)
        f.columnconfigure(1, weight=1)
        f.rowconfigure(0, weight=1)
        left = Card(f, "Trial setup", "Only combinations that assemble in simulation can be picked")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 14), pady=(14, 0))
        right = Card(f, "Output", "Preflight report (offline). The physical run opens in its own terminal.")
        right.grid(row=0, column=1, sticky="nsew", pady=(14, 0))
        b = left.body

        self.seg_variant = self._field(b, "Variant", lambda p: Segmented(p, self.variant, ["1", "2", "3", "4", "5"],
                                                                        {str(i): f"V{i}" for i in range(1, 6)}))
        self.seg_level = self._field(b, "Method", lambda p: Segmented(p, self.level, ["L2", "CAD", "L1", "L1D", "L0"]))
        self.level_info = tk.Label(b, text="", bg=CARD, fg=INK2, font=(FONT, 9, "italic"))
        self.level_info.pack(anchor="w", pady=(0, 12))
        self._field(b, "Timing", lambda p: Segmented(p, self.temporal, ["fixed", "cad"],
                                                     {"fixed": "fixed (demo)", "cad": "CAD-scaled"}))

        grid = tk.Frame(b, bg=CARD)
        grid.pack(fill="x", pady=(0, 6))
        for col, (lab, var, vals, w) in enumerate((("From", self.from_stage, ORDER, 14),
                                                   ("Until", self.until, ORDER, 14),
                                                   ("Transit rad/s", self.speed,
                                                    ["0.14", "0.20", "0.28", "0.30", "0.35"], 7))):
            cell = tk.Frame(grid, bg=CARD)
            cell.grid(row=0, column=col, sticky="w", padx=(0, 14))
            tk.Label(cell, text=lab.upper(), bg=CARD, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 4))
            ttk.Combobox(cell, textvariable=var, values=vals, state="readonly", width=w, font=(FONT, 10)).pack()
        tk.Label(b, text="Transit speed applies to REACH/TRANSPORT only; every other stage stays at 0.14 rad/s.",
                 bg=CARD, fg=MUTED, font=(FONT, 8)).pack(anchor="w", pady=(2, 10))

        ttk.Checkbutton(b, text="--auto  (type only RUN_V<n>_START and RUN_INSERT)", variable=self.auto).pack(anchor="w")
        ttk.Checkbutton(b, text="Reset MoveIt scene before the run (needed from HOME_START)",
                        variable=self.reset_scene).pack(anchor="w", pady=(2, 10))
        prow = tk.Frame(b, bg=CARD)
        prow.pack(fill="x", pady=(0, 10))
        tk.Label(prow, text="PLANS", bg=CARD, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 4))
        tk.Entry(prow, textvariable=self.plans, font=(MONO, 9), relief="solid", bd=1,
                 highlightthickness=0, fg=INK).pack(fill="x", ipady=4)

        self.cmd_label = tk.Label(b, text="", bg="#f1f4f8", fg=INK2, font=(MONO, 8), justify="left", anchor="w",
                                  wraplength=420, padx=10, pady=8)
        self.cmd_label.pack(fill="x", pady=(4, 12))
        btns = tk.Frame(b, bg=CARD)
        btns.pack(fill="x")
        self.btn_pre = Btn(btns, "1   Preflight  (no motion)", self.preflight, ACCENT, ACCENT_D)
        self.btn_pre.pack(side="left")
        self.btn_go = Btn(btns, "2   Run on robot", self.launch, GO, GO_D)
        self.btn_go.pack(side="left", padx=(10, 0))
        self.state = tk.Label(b, text="", bg=CARD, fg=INK2, font=(FONT, 9), wraplength=430, justify="left")
        self.state.pack(anchor="w", pady=(10, 0))

        self.out = tk.Text(right.body, wrap="word", bg=CONSOLE_BG, fg=CONSOLE_FG, insertbackground="white",
                           font=(MONO, 9), relief="flat", padx=12, pady=10)
        self.out.pack(fill="both", expand=True)
        for tag, col in (("pass", "#68d391"), ("fail", "#fc8181"), ("dim", "#7b8da0"), ("head", "#90cdf4")):
            self.out.tag_configure(tag, foreground=col)
        self._write(f"repository  {ROOT}\n", "dim")
        self._write("Pick a trial, then Preflight. Nothing here moves the robot.\n", "dim")

    def _variant_changed(self, *_):
        opts = ALLOWED[int(self.variant.get())]
        self.seg_level.set_enabled(opts)
        if self.level.get() not in opts:
            self.level.set("L2")

    def _args(self):
        n, speed = int(self.variant.get()), float(self.speed.get())
        if self.level.get() not in ALLOWED[n]:
            raise ValueError(f"{self.level.get()} is not allowed on V{n}.")
        if ORDER.index(self.from_stage.get()) > ORDER.index(self.until.get()):
            raise ValueError("'From' must not come after 'Until'.")
        a = ["python3", RUNNER, "--plans", self.plans.get().strip(), "--variant", str(n), "--level", self.level.get(),
             "--temporal", self.temporal.get(), "--transit-speed", f"{speed:g}"]
        if self.from_stage.get() != "HOME_START":
            a += ["--from", self.from_stage.get()]
        if self.until.get() != "HOME":
            a += ["--until", self.until.get()]
        if self.auto.get():
            a += ["--auto"]
        return a

    def _changed(self, *_):
        self.passed_cfg = None
        self.btn_go.enable(False)
        self.level_info.configure(text=LEVEL_INFO.get(self.level.get(), ""))
        try:
            a = self._args()
            self.cmd_label.configure(text="$ " + shlex.join(a[1:]) + "  [--execute]", fg=INK2)
            self.state.configure(text="Run the preflight for this exact setup to unlock the robot run.", fg=INK2)
        except ValueError as e:
            self.cmd_label.configure(text=str(e), fg=BAD)
        self._set_pill("NOT PREFLIGHTED", "#5a6b7d")

    def _write(self, s, tag=None):
        self.out.insert("end", s, tag)
        self.out.see("end")

    def preflight(self):
        if self.busy:
            return
        try:
            a = self._args()
        except ValueError as e:
            messagebox.showerror("Not allowed", str(e))
            return
        if not (ROOT / self.plans.get().strip()).is_dir():
            messagebox.showerror("Plans missing", f"{ROOT / self.plans.get().strip()} does not exist on this laptop.")
            return
        self.busy = True
        self.btn_pre.enable(False)
        self.out.delete("1.0", "end")
        self._write("OFFLINE PREFLIGHT  ·  no ROS, no motion\n", "head")
        self._write("$ " + shlex.join(a) + "\n\n", "dim")
        self.state.configure(text="Preflight running… (first run of a level after a calibration change can take "
                                  "a few minutes while it recomputes the plan)", fg=INK2)
        self._set_pill("PREFLIGHT…", ACCENT)

        def work():
            try:
                r = subprocess.run(a, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   timeout=PREFLIGHT_TIMEOUT_S)
                self.q.put((tuple(a), r.returncode, r.stdout))
            except Exception as e:              # noqa: BLE001  (shown to the operator)
                self.q.put((tuple(a), 1, f"{type(e).__name__}: {e}"))
        threading.Thread(target=work, daemon=True).start()
        self.after(150, self._poll)

    def _poll(self):
        try:
            cfg, code, text = self.q.get_nowait()
        except queue.Empty:
            self.after(150, self._poll)
            return
        self.busy = False
        self.btn_pre.enable(True)
        for line in text.splitlines():
            if "UserWarning" in line or "warnings.warn" in line:
                continue
            tag = "pass" if "PASS" in line and "FAIL" not in line else "fail" if any(
                k in line for k in ("FAIL", "REFUSED", "Error", "Traceback")) else None
            self._write(line + "\n", tag)
        if code == 0 and "PREFLIGHT PASS" in text and cfg == tuple(self._args()):
            self.passed_cfg = cfg
            self.btn_go.enable(True)
            self._set_pill("PREFLIGHT PASS", OK)
            self.state.configure(text="Preflight passed. 'Run on robot' opens the staged runner in a terminal.", fg=OK)
        else:
            self._set_pill("PREFLIGHT FAILED", BAD)
            self.state.configure(text="Preflight did not pass: do not run this on the robot. Read the output.", fg=BAD)

    def launch(self):
        a = self._args()
        if tuple(a) != self.passed_cfg:
            messagebox.showerror("Preflight needed", "Settings changed since the preflight. Run it again.")
            return
        n = self.variant.get()
        where = ("The robot must be at HOME with part V%s in its pocket and the fitting empty." % n
                 if self.from_stage.get() == "HOME_START" else
                 "Starting at %s: the robot, gripper and part must be exactly where the previous run stopped "
                 "(part still attached in MoveIt, so do NOT reset the scene)." % self.from_stage.get())
        if self.from_stage.get() != "HOME_START" and self.reset_scene.get():
            messagebox.showerror("Scene reset", "Untick 'Reset MoveIt scene' when continuing from %s: the reset "
                                 "detaches the held part and the runner will refuse." % self.from_stage.get())
            return
        if not messagebox.askyesno("Run on the real robot",
                                   f"V{n}  ·  {self.level.get()}  ·  {self.temporal.get()} timing"
                                   f"{'  ·  --auto' if self.auto.get() else ''}\n\n{where}\n\n"
                                   "Nobody in the workspace, pendant stop in reach?\n\n"
                                   "A terminal opens; answer the runner's prompts there.", icon="warning"):
            return
        term = shutil.which("gnome-terminal") or shutil.which("xterm")
        if not term:
            messagebox.showerror("No terminal", "Install gnome-terminal or xterm.")
            return
        steps = ["source /opt/ros/humble/setup.bash", 'source "$HOME/ros2_ws/install/setup.bash"',
                 "export ROS_DOMAIN_ID=0", "cd " + shlex.quote(str(ROOT))]
        if self.reset_scene.get():
            steps.append("python3 " + SCENE + " --reset --physical")
        script = (" && ".join(steps) + " && " + shlex.join(a + ["--execute"]) +
                  '; rc=$?; echo; if [ $rc -eq 0 ]; then echo "finished (exit 0)"; else echo "STOPPED or FAILED (exit $rc): read the lines above"; fi; read -r -p "Enter closes this terminal "')
        title = f"V{n} {self.level.get()} physical"
        cmd = ([term, "--title", title, "--", "bash", "-c", script] if Path(term).name == "gnome-terminal"
               else [term, "-T", title, "-e", "bash", "-c", script])
        subprocess.Popen(cmd, cwd=ROOT)
        self._changed()
        self._set_pill("RUNNING IN TERMINAL", GO)
        self.state.configure(text="Runner opened in a terminal. Watch the robot; answer the prompts there. "
                                  "Afterwards record the outcome under Observations.", fg=GO_D)

    # ---------- previous runs ----------
    def _runs_tab(self):
        f = self.tab_runs
        card = Card(f, "Previous runs", "Contact force = change from the stage start (gripper weight removed). "
                                        "Controller PASS is not proof of assembly: see Observations.")
        card.pack(fill="both", expand=True, pady=(14, 0))
        top = tk.Frame(card.body, bg=CARD)
        top.pack(fill="x", pady=(0, 10))
        self.only_plans = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Only runs with the plans above", variable=self.only_plans,
                        command=self.refresh_runs).pack(side="left")
        Btn(top, "Refresh", self.refresh_runs, ACCENT, ACCENT_D).pack(side="right")
        cols = ("run", "v", "lvl", "timing", "status", "ins", "rot", "plans")
        heads = {"run": "Run", "v": "Var", "lvl": "Method", "timing": "Timing", "status": "Runner",
                 "ins": "INSERT contact N", "rot": "ROTATE contact N", "plans": "Plans"}
        widths = {"run": 220, "v": 50, "lvl": 70, "timing": 70, "status": 85, "ins": 130, "rot": 135, "plans": 170}
        box = tk.Frame(card.body, bg=CARD)
        box.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(box, columns=cols, show="headings")
        for c in cols:
            self.tree.heading(c, text=heads[c])
            self.tree.column(c, width=widths[c], anchor="w" if c in ("run", "plans") else "center")
        self.tree.tag_configure("contact", foreground=WARN)
        self.tree.tag_configure("stopped", foreground=MUTED)
        self.tree.tag_configure("odd", background="#f8fafc")
        sb = ttk.Scrollbar(box, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.tree.bind("<Double-1>", self._to_notes)
        tk.Label(card.body, text="Orange = contact above 8 N.  Double-click a run to record its outcome.",
                 bg=CARD, fg=MUTED, font=(FONT, 9)).pack(anchor="w", pady=(8, 0))

    def refresh_runs(self):
        self.tree.delete(*self.tree.get_children())
        want = Path(self.plans.get().strip()).name
        names = []
        for i, p in enumerate(sorted((p for p in RUNS_DIR.glob("V*_*_20*") if p.is_dir()), reverse=True)):
            try:
                s = json.loads((p / "status.json").read_text())
            except (OSError, ValueError):
                s = {}
            plans = Path(str(s.get("plans", "")).rstrip("/")).name
            if self.only_plans.get() and plans != want:
                continue
            try:
                stages = {x["stage"]: x for x in json.loads((p / "wrench_summary.json").read_text()).get("stages", [])
                          if x.get("available")}
            except (OSError, ValueError):
                stages = {}
            g = lambda k: stages[k].get("max_abs_force_change_from_stage_start_N") if k in stages else None
            ins, rot = g("INSERT"), g("ROTATE")
            status = str(s.get("status", "?"))
            tags = ["odd"] if len(names) % 2 else []
            if status != "PASS":
                tags.append("stopped")
            elif max(ins or 0, rot or 0) > 8:
                tags.append("contact")
            self.tree.insert("", "end", values=(p.name, s.get("variant", p.name[:2]), s.get("level", "L2"),
                                                s.get("temporal", "fixed"), status,
                                                "–" if ins is None else f"{ins:.1f}", "–" if rot is None else f"{rot:.1f}",
                                                plans), tags=tags)
            names.append(p.name)
        self.run_names = names
        if hasattr(self, "note_sel"):
            self.note_sel["values"] = names
            if self.note_run.get() not in names and names:
                self.note_run.set(names[0])

    def _to_notes(self, _e):
        sel = self.tree.selection()
        if sel:
            self.note_run.set(self.tree.item(sel[0], "values")[0])
            self.nb.select(self.tab_notes)

    # ---------- observations ----------
    def _notes_tab(self):
        card = Card(self.tab_notes, "Physical outcome", "What the controller cannot see. Appends one row to "
                                                         f"{NOTES_FILE.relative_to(ROOT)}; robot logs are never changed.")
        card.pack(fill="both", expand=True, pady=(14, 0))
        b = card.body
        tk.Label(b, text="RUN", bg=CARD, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 4))
        self.note_run = tk.StringVar()
        self.note_sel = ttk.Combobox(b, textvariable=self.note_run, state="readonly", width=48, font=(FONT, 10))
        self.note_sel.pack(anchor="w", pady=(0, 14))
        grid = tk.Frame(b, bg=CARD)
        grid.pack(anchor="w")
        self.nv = {}
        for i, (k, lab) in enumerate((("seated", "Seated (20.0 mm)?"), ("locked", "Locked after release?"),
                                      ("contact", "Contact / catching?"), ("plate_lift", "Plate lifted or moved?"),
                                      ("assistance", "Touched / assisted?"))):
            cell = tk.Frame(grid, bg=CARD)
            cell.grid(row=i // 3, column=i % 3, sticky="w", padx=(0, 26), pady=(0, 12))
            tk.Label(cell, text=lab.upper(), bg=CARD, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w", pady=(0, 4))
            v = tk.StringVar(value="—")
            self.nv[k] = v
            Segmented(cell, v, ["yes", "no", "—"]).pack(anchor="w")
        row = tk.Frame(b, bg=CARD)
        row.pack(anchor="w", pady=(4, 12))
        tk.Label(row, text="PART TOP ABOVE FITTING TOP (MM)", bg=CARD, fg=MUTED, font=(FONT, 8, "bold")).pack(anchor="w")
        self.top_mm = tk.StringVar()
        tk.Entry(row, textvariable=self.top_mm, width=10, font=(FONT, 11), relief="solid", bd=1).pack(anchor="w",
                                                                                                    pady=(4, 0), ipady=3)
        tk.Label(b, text="NOTES (photos, where it caught, anything unusual)", bg=CARD, fg=MUTED,
                 font=(FONT, 8, "bold")).pack(anchor="w")
        self.note_text = tk.Text(b, height=5, wrap="word", font=(FONT, 10), relief="solid", bd=1, padx=8, pady=6)
        self.note_text.pack(fill="x", pady=(4, 14))
        Btn(b, "Save observation", self.save_note, ACCENT, ACCENT_D).pack(anchor="w")
        self.note_msg = tk.Label(b, text="", bg=CARD, fg=OK, font=(FONT, 9))
        self.note_msg.pack(anchor="w", pady=(8, 0))

    def save_note(self):
        run = self.note_run.get()
        if run not in getattr(self, "run_names", []):
            messagebox.showerror("No run", "Pick a run folder first.")
            return
        top = self.top_mm.get().strip()
        if top:
            try:
                float(top)
            except ValueError:
                messagebox.showerror("Height", "Part-top height must be a number (mm).")
                return
        parts = run.split("_")
        row = {"time": datetime.now().isoformat(timespec="seconds"), "run": run, "variant": parts[0],
               "level": parts[1], "part_top_mm": top, "notes": self.note_text.get("1.0", "end").strip(),
               **{k: ("" if v.get() == "—" else v.get()) for k, v in self.nv.items()}}
        NOTES_FILE.parent.mkdir(parents=True, exist_ok=True)
        new = not NOTES_FILE.exists() or NOTES_FILE.stat().st_size == 0
        with NOTES_FILE.open("a", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=NOTE_FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)
        self.note_msg.configure(text=f"Saved {run} at {row['time'][11:]}")
        for v in self.nv.values():
            v.set("—")
        self.top_mm.set("")
        self.note_text.delete("1.0", "end")


if __name__ == "__main__":
    TrialGUI().mainloop()
