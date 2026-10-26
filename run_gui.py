from __future__ import annotations
import csv
import copy
import json
import math
import os
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from usv_colav.geometry import NM_TO_M, bearing_deg, m_to_nm, relative_bearing, wrap360
from usv_colav.models import StaticObstacle, Vessel
from usv_colav.scenarios import PRESETS, TargetSpec, build_random_scenario, build_scenario
from usv_colav.simulator import SimConfig, USVSimulator
from usv_colav.rule_policy import choose_rule_action
from usv_colav.case_loader import list_encounter_cases, load_encounter_csv

APP_DIR = Path(__file__).resolve().parent
UNLIMITED_PLAYBACK_LABEL = "跟随计算速度（不限速）"
UNLIMITED_STEPS_PER_TICK = 1  # exactly one simulation step per displayed animation frame


def playback_tick_plan(value: str) -> tuple[int, int]:
    """Return (simulation_steps_per_displayed_frame, delay_ms).

    Every mode advances exactly one simulator step per animation callback. Accelerated
    playback shortens the interval between frames; it never batches multiple steps into
    one visible frame. Unlimited mode has no intentional delay and yields between frames
    so Tk can process canvas redraws and user input.
    """
    text = str(value).strip()
    if "不限速" in text or text.lower() in {"unlimited", "max", "fastest"}:
        return 1, 0
    try:
        speed = float(text.lower().replace("x", "").strip())
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError
    except (TypeError, ValueError):
        speed = 1.0
    # Retain the established 1x interval (180 ms) but change only the per-frame
    # delay as speed changes. One call always advances one simulation step.
    return 1, max(1, int(round(180.0 / speed)))

class RunLogger:
    FIELDS = ["timestamp", "step", "elapsed_s", "own_east_m", "own_north_m", "own_course_deg",
              "own_speed_kn", "course_delta_deg", "speed_ratio", "min_range_m", "metrics_json",
              "positions_json", "collision", "sda_violations", "terminal_reason", "active_failure",
              "visible_targets", "sensor_json", "environment_json", "obstacle_contacts", "obstacle_clearances_json"]
    def __init__(self, folder: Path):
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = folder / f"manual_run_{stamp}.csv"
        self.file = self.path.open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()
        self.file.flush()
        self._rows_since_flush = 0

    def write(self, sim: USVSimulator):
        state = sim.state_dict()
        minimum = min(sim.min_distance_by_target.values(), default=float("inf"))
        self.writer.writerow({
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "step": sim.step_count, "elapsed_s": round(sim.elapsed_s, 3),
            "own_east_m": round(sim.own.east_m, 3), "own_north_m": round(sim.own.north_m, 3),
            "own_course_deg": round(sim.own.course_deg, 3), "own_speed_kn": round(sim.own.speed_kn, 3),
            "course_delta_deg": sim.last_course_delta_deg, "speed_ratio": sim.last_speed_ratio,
            "min_range_m": round(minimum, 3),
            "metrics_json": json.dumps(state["metrics"], ensure_ascii=False),
            "positions_json": json.dumps({"own": state["own"], "targets": state["targets"]}, ensure_ascii=False),
            "collision": int(sim.collision), "sda_violations": ";".join(sorted(sim.sda_violations)),
            "terminal_reason": sim.terminal_reason, "active_failure": sim.active_failure,
            "visible_targets": state["sensor"].get("visible_targets", 0),
            "sensor_json": json.dumps(state["sensor"], ensure_ascii=False),
            "environment_json": json.dumps(state["environment"], ensure_ascii=False),
            "obstacle_contacts": ";".join(sorted(sim.obstacle_contacts)),
            "obstacle_clearances_json": json.dumps(state["obstacle_clearances_m"], ensure_ascii=False),
        })
        # Frequent flush() calls materially slow the unlimited-speed playback mode.
        # CSV rows are still written on every simulator step and flushed periodically.
        self._rows_since_flush += 1
        if self._rows_since_flush >= 25:
            self.file.flush()
            self._rows_since_flush = 0

    def close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()


class USVColavApp:
    PRESET_LABELS = {
        "自定义（使用下方参数）": "custom",
        "对遇测试": "head_on",
        "右舷交叉测试": "crossing_starboard",
        "左舷交叉测试": "crossing_port",
        "追越测试": "overtaking",
        "CCS 附录 D.5 参数（2目标）": "ccs_d5",
        "CCS 附录 D.6 参数（取 TV1-TV3）": "ccs_d6_3",
        "随机危险场景": "random",
    }

    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("无人艇自动避碰仿真与原始测试用例回放（CCS 参考原型）")
        root.geometry("1460x900")
        root.minsize(1180, 760)
        self.sim: USVSimulator | None = None
        self.logger: RunLogger | None = None
        self.running = False
        self.after_id = None
        self.ppo_model = None
        self.ppo_path = "models/usv_colav_ppo.zip"
        self.sim_count = 0
        self.last_alert_score = 0
        self.timeline_history: list[dict] = []
        self.timeline_cursor = 0
        self._updating_timeline = False
        self.test_case_items: list[tuple[str, Path]] = []
        self.current_test_case: tuple[str, Path] | None = None
        self.status_var = tk.StringVar(value="准备就绪：先选择场景/设置目标船参数，再点击“重置场景”。")
        self.timeline_index_var = tk.DoubleVar(value=0.0)
        self.timeline_label_var = tk.StringVar(value="t=0 s / 0 s")
        self.policy_var = tk.StringVar(value="规则基线（无需训练）")
        self.preset_var = tk.StringVar(value="对遇测试")
        self.count_var = tk.StringVar(value="1V1")
        self.own_speed_var = tk.StringVar(value="15")
        self.own_course_var = tk.StringVar(value="0")
        self.sda_var = tk.StringVar(value="0.30")
        self.dt_var = tk.StringVar(value="5")
        self.environment_var = tk.StringVar(value="正常海况（传感器有小误差）")
        self.failure_step_var = tk.StringVar(value="20")
        self.model_path_var = tk.StringVar(value=self.ppo_path)
        self.case_group_var = tk.StringVar(value="全部")
        self.case_skip_seconds_var = tk.StringVar(value="1800")
        self.playback_speed_var = tk.StringVar(value="1x")
        self.case_count_var = tk.StringVar(value="用例加载中…")
        self.target_entries: list[dict[str, tk.Entry]] = []
        self.target_values = [
            {"speed": "12", "course": "180", "range": "3.0", "bearing": "0", "sda": ""},
            {"speed": "10", "course": "0", "range": "2.8", "bearing": "60", "sda": ""},
            {"speed": "14", "course": "260", "range": "3.5", "bearing": "-55", "sda": ""},
        ]
        self._build_ui()
        self._apply_preset()
        self.reset_simulation()
        self.root.protocol("WM_DELETE_WINDOW", self.close_app)

    def _build_ui(self):
        header = ttk.Frame(self.root, padding=(10, 8))
        header.pack(fill="x")
        ttk.Label(header, text="USV COLAV | 多船会遇 + CPA/TCPA + PPO + 268 用例回放", font=("Microsoft YaHei UI", 16, "bold")).pack(side="left")
        ttk.Label(header, text="GB/T 46851—2025 参考仿真；非实船控制软件", foreground="#9a3412").pack(side="right")

        content = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        content.pack(fill="both", expand=True)
        left = ttk.Frame(content)
        left.pack(side="left", fill="both", expand=True, padx=(0, 8))
        right = ttk.Frame(content, width=450)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)

        self.canvas = tk.Canvas(left, bg="#f7fafc", highlightthickness=1, highlightbackground="#94a3b8", width=900, height=680)
        self.canvas.pack(fill="both", expand=True)
        timeline_panel = ttk.LabelFrame(left, text="场景态势时间进度（拖动查看任一仿真步）", padding=(6, 3))
        timeline_panel.pack(fill="x", pady=(4, 0))
        self.timeline_scale = ttk.Scale(
            timeline_panel, from_=0, to=0, orient="horizontal", variable=self.timeline_index_var,
            command=self._on_timeline_drag,
        )
        self.timeline_scale.pack(side="left", fill="x", expand=True, padx=(2, 8))
        self.timeline_scale.bind("<ButtonRelease-1>", self._seek_timeline)
        self.timeline_scale.bind("<KeyRelease>", self._seek_timeline)
        ttk.Label(timeline_panel, textvariable=self.timeline_label_var, width=25, anchor="e").pack(side="right")
        footer = ttk.Frame(left, padding=(2, 8))
        footer.pack(fill="x")
        ttk.Label(footer, textvariable=self.status_var, wraplength=850).pack(anchor="w")

        nb = ttk.Notebook(right)
        nb.pack(fill="both", expand=True)
        settings = ttk.Frame(nb, padding=10)
        analysis = ttk.Frame(nb, padding=10)
        replay = ttk.Frame(nb, padding=8)
        nb.add(settings, text="场景与控制")
        nb.add(analysis, text="态势 / 风险")
        nb.add(replay, text="测试用例回放")

        ttk.Label(settings, text="场景预设").grid(row=0, column=0, sticky="w", pady=3)
        self.preset_combo = ttk.Combobox(settings, textvariable=self.preset_var, state="readonly", width=28,
                                         values=list(self.PRESET_LABELS.keys()))
        self.preset_combo.grid(row=0, column=1, sticky="ew", pady=3)
        self.preset_combo.bind("<<ComboboxSelected>>", lambda _e: self._apply_preset())

        ttk.Label(settings, text="目标船数量").grid(row=1, column=0, sticky="w", pady=3)
        self.count_combo = ttk.Combobox(settings, textvariable=self.count_var, state="readonly", width=10, values=["1V1", "1V2", "1V3"])
        self.count_combo.grid(row=1, column=1, sticky="w", pady=3)
        ttk.Label(settings, text="格式：1Vn = 1 艘本船 + n 艘目标船", foreground="#475569").grid(row=2, column=0, columnspan=2, sticky="w")

        own_frame = ttk.LabelFrame(settings, text="本船 OV", padding=8)
        own_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 5))
        self._add_field(own_frame, "航速 kn", self.own_speed_var, 0, 0, 8)
        self._add_field(own_frame, "航向 °", self.own_course_var, 0, 2, 8)
        self._add_field(own_frame, "SDA 海里", self.sda_var, 1, 0, 8)
        self._add_field(own_frame, "步长 s", self.dt_var, 1, 2, 8)

        target_frame = ttk.LabelFrame(settings, text="目标船参数（相对方位：右舷为正，左舷为负）", padding=6)
        target_frame.grid(row=4, column=0, columnspan=2, sticky="ew", pady=5)
        headers = ["目标", "航速", "航向°", "距离NM", "相对方位°", "SDA NM* "]
        for col, header_text in enumerate(headers):
            ttk.Label(target_frame, text=header_text).grid(row=0, column=col, padx=2, pady=2)
        widths = [4, 6, 6, 6, 7, 6]
        for i in range(3):
            ttk.Label(target_frame, text=f"TV{i+1}").grid(row=i+1, column=0, padx=2, pady=3)
            entry_row = {}
            for col, key in enumerate(("speed", "course", "range", "bearing", "sda"), start=1):
                e = ttk.Entry(target_frame, width=widths[col], justify="center")
                e.grid(row=i+1, column=col, padx=2, pady=3)
                e.insert(0, self.target_values[i][key])
                entry_row[key] = e
            self.target_entries.append(entry_row)

        env_frame = ttk.LabelFrame(settings, text="环境扰动 / 系统失效注入", padding=6)
        env_frame.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(6, 4))
        ttk.Label(env_frame, text="测试工况").grid(row=0, column=0, sticky="w", padx=2, pady=2)
        self.environment_combo = ttk.Combobox(env_frame, textvariable=self.environment_var, state="readonly", width=26,
            values=["正常海况（传感器有小误差）", "风浪流扰动", "受限能见度", "强传感器噪声", "GNSS 失效", "罗经偏差", "推进失效", "舵机卡滞", "传感器掉线", "静态障碍物"])
        self.environment_combo.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        ttk.Label(env_frame, text="故障触发步数").grid(row=1, column=0, sticky="w", padx=2, pady=2)
        ttk.Entry(env_frame, textvariable=self.failure_step_var, width=8).grid(row=1, column=1, sticky="w", padx=2, pady=2)
        ttk.Label(env_frame, text="风浪流/障碍/故障仅用于仿真压力测试，不代表实海况模型。", foreground="#9a3412").grid(row=2, column=0, columnspan=2, sticky="w", padx=2, pady=2)

        ttk.Label(settings, text="策略选择").grid(row=6, column=0, sticky="w", pady=(8, 3))
        self.policy_combo = ttk.Combobox(settings, textvariable=self.policy_var, state="readonly", width=28,
                                         values=["规则基线（无需训练）", "PPO 模型（先训练）"])
        self.policy_combo.grid(row=6, column=1, sticky="ew", pady=(8, 3))
        ttk.Label(settings, text="PPO 模型路径").grid(row=7, column=0, sticky="w", pady=3)
        model_frame = ttk.Frame(settings)
        model_frame.grid(row=7, column=1, sticky="ew", pady=3)
        ttk.Entry(model_frame, textvariable=self.model_path_var, width=22).pack(side="left", fill="x", expand=True)
        ttk.Button(model_frame, text="浏览", command=self.choose_model).pack(side="left", padx=(4, 0))
        ttk.Button(settings, text="加载 PPO 模型", command=self.load_ppo).grid(row=8, column=1, sticky="e", pady=4)

        control = ttk.LabelFrame(settings, text="仿真控制", padding=8)
        control.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(8, 4))
        ttk.Button(control, text="重置场景", command=self.reset_simulation).grid(row=0, column=0, padx=3, pady=4, sticky="ew")
        self.start_btn = ttk.Button(control, text="开始运行", command=self.toggle_running)
        self.start_btn.grid(row=0, column=1, padx=3, pady=4, sticky="ew")
        ttk.Button(control, text="单步运行", command=self.step_once).grid(row=1, column=0, padx=3, pady=4, sticky="ew")
        ttk.Button(control, text="停止 / 保存日志", command=self.stop_and_save).grid(row=1, column=1, padx=3, pady=4, sticky="ew")
        ttk.Button(control, text="退出系统", command=self.close_app).grid(row=2, column=0, columnspan=2, padx=3, pady=4, sticky="ew")
        control.columnconfigure(0, weight=1); control.columnconfigure(1, weight=1)

        notes = ("提示：目标船参数在场景重置时生效。航向角：0°北、90°东、顺时针增加。\n"
                 "单船 SDA 可选填写；留空使用本船的默认 SDA。目标船默认保向保速。\n"
                 "本工程不输出舵机/电机硬件控制指令。")
        ttk.Label(settings, text=notes, wraplength=400, foreground="#475569").grid(row=10, column=0, columnspan=2, sticky="w", pady=(8, 0))
        settings.columnconfigure(1, weight=1)

        ttk.Label(analysis, text="本船状态", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        self.own_info = tk.Text(analysis, height=8, width=50, wrap="word", state="disabled", bg="#f8fafc")
        self.own_info.pack(fill="x", pady=(3, 10))
        ttk.Label(analysis, text="逐目标船态势（CPA / TCPA / SDA）", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        self.target_info = tk.Text(analysis, height=18, width=50, wrap="word", state="disabled", bg="#f8fafc")
        self.target_info.pack(fill="both", expand=True, pady=(3, 8))
        ttk.Label(analysis, text="风险等级为本工程近似指标；GB/T 46851 附录A给出的是特定会遇局面的量化示例。",
                  wraplength=400, foreground="#9a3412").pack(anchor="w")

        self._build_test_case_tab(replay)

    def _build_test_case_tab(self, parent: ttk.Frame):
        """Create GUI for browsing and replaying the packaged original CSV cases."""
        ttk.Label(parent, text="原始测试用例（共 268 个）", font=("Microsoft YaHei UI", 11, "bold")).pack(anchor="w")
        ttk.Label(parent, text="选择 1V1 / 1V2 / 1V3 中的原始 CSV；双击也可直接加载。",
                  wraplength=405, foreground="#475569").pack(anchor="w", pady=(2, 5))

        filter_row = ttk.Frame(parent)
        filter_row.pack(fill="x", pady=(0, 4))
        ttk.Label(filter_row, text="分组").pack(side="left")
        group_combo = ttk.Combobox(filter_row, textvariable=self.case_group_var, state="readonly", width=10,
                                   values=["全部", "1V1", "1V2", "1V3"])
        group_combo.pack(side="left", padx=(4, 8))
        group_combo.bind("<<ComboboxSelected>>", lambda _event: self._refresh_case_list())
        ttk.Button(filter_row, text="刷新列表", command=self._refresh_case_list).pack(side="left")
        ttk.Label(filter_row, textvariable=self.case_count_var, foreground="#475569").pack(side="right")

        list_frame = ttk.Frame(parent)
        list_frame.pack(fill="x", expand=False)
        self.case_listbox = tk.Listbox(list_frame, height=11, exportselection=False, activestyle="dotbox",
                                       font=("Consolas", 9))
        case_scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self.case_listbox.yview)
        self.case_listbox.configure(yscrollcommand=case_scroll.set)
        self.case_listbox.pack(side="left", fill="both", expand=True)
        case_scroll.pack(side="right", fill="y")
        self.case_listbox.bind("<<ListboxSelect>>", lambda _event: self._preview_selected_case())
        self.case_listbox.bind("<Double-Button-1>", lambda _event: self._load_selected_test_case())

        case_buttons = ttk.Frame(parent)
        case_buttons.pack(fill="x", pady=5)
        ttk.Button(case_buttons, text="加载 / 重新开始", command=self._load_selected_test_case).pack(side="left", fill="x", expand=True, padx=(0, 3))
        ttk.Button(case_buttons, text="上一个", command=lambda: self._move_case_selection(-1)).pack(side="left", padx=2)
        ttk.Button(case_buttons, text="下一个", command=lambda: self._move_case_selection(1)).pack(side="left", padx=(2, 0))

        ttk.Label(parent, text="所选用例信息", font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w", pady=(3, 2))
        detail_frame = ttk.Frame(parent)
        detail_frame.pack(fill="both", expand=True)
        self.case_info = tk.Text(detail_frame, height=10, wrap="word", state="disabled", bg="#f8fafc", font=("Microsoft YaHei UI", 9))
        detail_scroll = ttk.Scrollbar(detail_frame, orient="vertical", command=self.case_info.yview)
        self.case_info.configure(yscrollcommand=detail_scroll.set)
        self.case_info.pack(side="left", fill="both", expand=True)
        detail_scroll.pack(side="right", fill="y")

        playback = ttk.LabelFrame(parent, text="播放与时间快进", padding=6)
        playback.pack(fill="x", pady=(6, 0))
        speed_row = ttk.Frame(playback)
        speed_row.pack(fill="x", pady=2)
        ttk.Label(speed_row, text="播放倍速").pack(side="left")
        self.playback_speed_combo = ttk.Combobox(
            speed_row, textvariable=self.playback_speed_var, state="readonly", width=23,
            values=["0.25x", "0.5x", "1x", "2x", "5x", "10x", "20x", "50x", UNLIMITED_PLAYBACK_LABEL])
        self.playback_speed_combo.pack(side="left", padx=(6, 10))
        ttk.Label(speed_row, text="运行中可调整").pack(side="left")
        skip_row = ttk.Frame(playback)
        skip_row.pack(fill="x", pady=2)
        ttk.Label(skip_row, text="快进仿真秒数").pack(side="left")
        ttk.Entry(skip_row, textvariable=self.case_skip_seconds_var, width=10).pack(side="left", padx=6)
        ttk.Button(skip_row, text="快进（仍计算控制）", command=self._fast_forward_simulated_time).pack(side="left", fill="x", expand=True)
        ttk.Label(playback, text="所有倍速均逐个仿真步刷新画面，不跳过中间帧；倍速越高，帧间等待越短。不限速模式不设置人为等待。",
                  wraplength=405, foreground="#166534").pack(anchor="w", pady=(3, 0))
        ttk.Label(playback, text="快进会逐步执行控制器、船舶运动和安全筛查并写入日志，只跳过中间逐帧绘图；按仿真步长取整。",
                  wraplength=405, foreground="#9a3412").pack(anchor="w", pady=(2, 0))
        self._refresh_case_list()

    def _refresh_case_list(self):
        if not hasattr(self, "case_listbox"):
            return
        group_label = self.case_group_var.get()
        group = "all" if group_label == "全部" else group_label
        try:
            self.test_case_items = list_encounter_cases(APP_DIR / "test_cases", group)
        except Exception as exc:
            self.test_case_items = []
            self.case_count_var.set("读取失败")
            messagebox.showerror("读取测试用例失败", str(exc))
            return
        self.case_listbox.delete(0, "end")
        for case_group, path in self.test_case_items:
            self.case_listbox.insert("end", f"{case_group}  |  {path.stem}")
        self.case_count_var.set(f"{len(self.test_case_items)} 个")
        if self.test_case_items:
            if self.current_test_case:
                current_key = (self.current_test_case[0], self.current_test_case[1].name)
                found = next((i for i, (g, p) in enumerate(self.test_case_items) if (g, p.name) == current_key), 0)
            else:
                found = 0
            self.case_listbox.selection_set(found)
            self.case_listbox.see(found)
            self._preview_selected_case()

    def _selected_test_case(self) -> tuple[str, Path] | None:
        selection = self.case_listbox.curselection() if hasattr(self, "case_listbox") else ()
        if not selection:
            return None
        index = int(selection[0])
        return self.test_case_items[index] if 0 <= index < len(self.test_case_items) else None

    def _move_case_selection(self, offset: int):
        if not self.test_case_items:
            return
        selection = self.case_listbox.curselection()
        current = int(selection[0]) if selection else 0
        new_index = min(len(self.test_case_items) - 1, max(0, current + offset))
        self.case_listbox.selection_clear(0, "end")
        self.case_listbox.selection_set(new_index)
        self.case_listbox.see(new_index)
        self._preview_selected_case()

    def _preview_selected_case(self):
        selected = self._selected_test_case()
        if not selected:
            self._set_case_info("没有选中测试用例。")
            return
        group, path = selected
        try:
            own, targets, meta = load_encounter_csv(path)
            rows = meta.get("source_rows", [])
            lines = [
                f"用例：{group}/{path.name}",
                f"目标船数量：{len(targets)}（1 本船 + {len(targets)} 目标船）",
                f"原始会遇类型：{meta.get('meeting_situations') or 'CSV 未填写'}",
                f"阶段：{' | '.join(r.get('stage', '') for r in rows if r.get('stage')) or '未填写'}",
                f"本船初始：SOG={own.speed_kn:.2f} kn，COG={own.course_deg:.2f}°",
                f"本船参考坐标：纬度 {meta['ownship_lat']:.6f}，经度 {meta['ownship_lon']:.6f}",
                "", "目标船初始参数："
            ]
            for idx, target in enumerate(targets):
                row = rows[idx] if idx < len(rows) else {}
                distance_nm = math.hypot(target.east_m - own.east_m, target.north_m - own.north_m) / NM_TO_M
                rb = relative_bearing(own.position, own.course_deg, target.position)
                dcpa = row.get("source_dcpa_nm")
                tcpa = row.get("source_tcpa_min")
                dcpa_text = f"{dcpa:.3f} NM" if dcpa is not None else "未填写"
                tcpa_text = f"{tcpa:.2f} min" if tcpa is not None else "未填写"
                lines.append(
                    f"{target.name}: SOG={target.speed_kn:.2f} kn，COG={target.course_deg:.2f}°，"
                    f"初始距离={distance_nm:.3f} NM，相对方位={rb:+.1f}°；"
                    f"原 CSV DCPA={dcpa_text}，TCPA={tcpa_text}"
                )
            lines += ["", "双击列表项或点击‘加载 / 重新开始’即可从此 CSV 的原始初始状态复现。"]
            self._set_case_info("\n".join(lines))
        except Exception as exc:
            self._set_case_info(f"读取失败：{exc}")

    def _set_case_info(self, text: str):
        if not hasattr(self, "case_info"):
            return
        self.case_info.configure(state="normal")
        self.case_info.delete("1.0", "end")
        self.case_info.insert("1.0", text)
        self.case_info.configure(state="disabled")

    def _load_selected_test_case(self):
        selected = self._selected_test_case()
        if not selected:
            messagebox.showwarning("未选择用例", "请先从列表中选择一个测试用例。")
            return
        group, path = selected
        try:
            own, targets, meta = load_encounter_csv(path)
            dt_s = float(self.dt_var.get())
            sda_nm = float(self.sda_var.get())
            if dt_s <= 0 or sda_nm <= 0:
                raise ValueError("仿真步长和 SDA 必须大于 0。")
            # Reflect the original case data in the ordinary scene-parameter panel.
            self.count_var.set(group)
            self.own_speed_var.set(f"{own.speed_kn:.3f}")
            self.own_course_var.set(f"{own.course_deg:.3f}")
            self.preset_var.set("自定义（使用下方参数）")
            for idx in range(3):
                entry_row = self.target_entries[idx]
                entry_row["sda"].delete(0, "end")
                if idx < len(targets):
                    target = targets[idx]
                    dist_nm = math.hypot(target.east_m-own.east_m, target.north_m-own.north_m) / NM_TO_M
                    brg = relative_bearing(own.position, own.course_deg, target.position)
                    values = {"speed": target.speed_kn, "course": target.course_deg,
                              "range": dist_nm, "bearing": brg}
                    for key, value in values.items():
                        entry_row[key].delete(0, "end")
                        entry_row[key].insert(0, f"{value:.4f}")

            if self.running:
                self.running = False
            self._cancel_animation_callback()
            self.start_btn.configure(text="开始运行")
            if self.logger:
                self.logger.close()
                self.logger = None

            seed_offset = {"1V1": 11000, "1V2": 22000, "1V3": 33000}[group]
            import re
            match = re.search(r"scene_(\d+)", path.stem)
            scene_number = int(match.group(1)) if match else 0
            cfg = SimConfig(dt_s=dt_s, sda_nm=sda_nm, max_steps=1800, safe_resolution_early_stop=False,
                            require_route_recovery=True,
                            random_seed=7 + seed_offset + scene_number)
            env_name = self.environment_var.get()
            if env_name == "风浪流扰动":
                cfg.current_speed_kn, cfg.current_dir_deg = 1.5, 45.0
                cfg.wind_speed_kn, cfg.wind_from_deg = 20.0, 270.0
                cfg.wave_height_m, cfg.wave_period_s = 2.0, 7.5
            elif env_name == "受限能见度":
                cfg.visibility_m = 1000.0
                cfg.sensor_position_sigma_m, cfg.sensor_course_sigma_deg = 12.0, 3.5
            elif env_name == "强传感器噪声":
                cfg.sensor_position_sigma_m, cfg.sensor_course_sigma_deg, cfg.sensor_speed_sigma_kn = 25.0, 5.0, 1.0
                cfg.sensor_dropout_probability = 0.12
            elif env_name in {"GNSS 失效", "罗经偏差", "推进失效", "舵机卡滞", "传感器掉线"}:
                try:
                    cfg.failure_at_step = max(0, int(self.failure_step_var.get()))
                except ValueError as exc:
                    raise ValueError("故障触发步数必须是非负整数。") from exc
                cfg.failure_mode = {"GNSS 失效": "gnss_loss", "罗经偏差": "compass_bias",
                    "推进失效": "propulsion_loss", "舵机卡滞": "rudder_jam", "传感器掉线": "sensor_dropout"}[env_name]
            obstacles = []
            if env_name == "静态障碍物":
                h = math.radians(own.course_deg)
                ahead = 1800.0
                obstacles = [StaticObstacle("OB1", own.east_m + math.sin(h)*ahead,
                    own.north_m + math.cos(h)*ahead, radius_m=100.0)]
            self.sim = USVSimulator(own, targets, cfg,
                f"原始测试用例 {group}/{path.name}；{meta.get('meeting_situations', '')}；工况={env_name}",
                obstacles=obstacles)
            self._reset_timeline_history()
            self.last_alert_score = 0
            self.current_test_case = (group, path)
            self.logger = RunLogger(APP_DIR / "logs")
            self.sim_count += 1
            self.status_var.set(
                f"已加载原始用例 {group}/{path.name}：1 本船 + {len(targets)} 目标船；"
                f"t=0s；{env_name}；日志={self.logger.path.name}"
            )
            self.paint()
            self.update_info()
            self._set_case_info(self._case_info_text(group, path, own, targets, meta, loaded=True))
        except Exception as exc:
            messagebox.showerror("加载测试用例失败", str(exc))

    def _case_info_text(self, group, path, own, targets, meta, loaded=False) -> str:
        rows = meta.get("source_rows", [])
        lines = [f"{'当前已加载' if loaded else '预览'}：{group}/{path.name}",
                 f"本船：{own.speed_kn:.2f} kn，航向 {own.course_deg:.2f}°；目标船数 {len(targets)}",
                 f"会遇类型：{meta.get('meeting_situations') or 'CSV 未填写'}", ""]
        for idx, target in enumerate(targets):
            row = rows[idx] if idx < len(rows) else {}
            distance_nm = math.hypot(target.east_m-own.east_m, target.north_m-own.north_m) / NM_TO_M
            rb = relative_bearing(own.position, own.course_deg, target.position)
            lines.append(f"{target.name}: {target.speed_kn:.2f} kn / COG {target.course_deg:.1f}° / {distance_nm:.2f} NM / 方位 {rb:+.1f}°")
            if row:
                lines.append(f"  原始 DCPA={row.get('source_dcpa_nm')} NM，TCPA={row.get('source_tcpa_min')} min，阶段={row.get('stage','')}")
        lines += ["", "快进会执行完整控制与运动学步骤；最终仿真时间以步长的整数倍推进。"]
        return "\n".join(lines)

    def _add_field(self, parent, label, var, row, col, width=8):
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w", padx=(0, 3), pady=3)
        ttk.Entry(parent, textvariable=var, width=width).grid(row=row, column=col+1, sticky="w", padx=(0, 7), pady=3)

    def _current_target_specs(self, count: int) -> list[TargetSpec]:
        specs = []
        for i in range(count):
            row = self.target_entries[i]
            try:
                speed = float(row["speed"].get())
                course = float(row["course"].get()) % 360
                dist = float(row["range"].get())
                bearing = float(row["bearing"].get())
                sda_text = row["sda"].get().strip()
                target_sda = float(sda_text) if sda_text else None
            except ValueError as exc:
                raise ValueError(f"TV{i+1} 存在非数字参数，请检查航速/航向/距离/相对方位。") from exc
            if speed < 0 or dist <= 0 or (target_sda is not None and target_sda <= 0):
                raise ValueError(f"TV{i+1} 航速不能为负数，距离及单船 SDA（如填写）必须大于 0。")
            specs.append(TargetSpec(speed, course, dist, bearing, 8.0, f"TV{i+1}", target_sda))
        return specs

    def _apply_preset(self):
        label = self.preset_var.get()
        key = self.PRESET_LABELS.get(label, "head_on")
        if key == "custom":
            return
        try:
            if key == "random":
                own, targets, _ = build_random_scenario(3)
                specs = []
                for t in targets:
                    dx, dy = t.east_m, t.north_m
                    specs.append(TargetSpec(t.speed_kn, t.course_deg, math.hypot(dx, dy) / NM_TO_M,
                                            relative_bearing(own.position, own.course_deg, t.position), t.length_m, t.name))
                preset_own_speed, preset_course = own.speed_kn, own.course_deg
            else:
                spec = PRESETS[key]
                specs = list(spec.targets)
                preset_own_speed, preset_course = spec.own_speed_kn, spec.own_course_deg
            self.own_speed_var.set(f"{preset_own_speed:.2f}")
            self.own_course_var.set(f"{preset_course:.1f}")
            self.count_var.set(f"1V{len(specs)}")
            for i in range(3):
                vals = specs[i] if i < len(specs) else TargetSpec(10 + i, (90 + i*60) % 360, 3.0, -45 + i*40)
                row = self.target_entries[i]
                values = (("speed", vals.speed_kn), ("course", vals.course_deg), ("range", vals.range_nm),
                          ("bearing", vals.bearing_rel_deg), ("sda", vals.sda_nm))
                for k, v in values:
                    row[k].delete(0, "end")
                    if v is not None:
                        row[k].insert(0, f"{v:.2f}")
        except Exception as exc:
            messagebox.showerror("预设加载失败", str(exc))

    def _parse_count(self) -> int:
        value = self.count_var.get()
        return int(value[-1]) if value in {"1V1", "1V2", "1V3"} else 1

    def reset_simulation(self):
        self.running = False
        self._cancel_animation_callback()
        self.start_btn.configure(text="开始运行")
        if self.logger:
            self.logger.close(); self.logger = None
        try:
            count = self._parse_count()
            own_speed = float(self.own_speed_var.get())
            own_course = float(self.own_course_var.get()) % 360
            sda_nm = float(self.sda_var.get())
            dt_s = float(self.dt_var.get())
            if own_speed <= 0 or sda_nm <= 0 or dt_s <= 0:
                raise ValueError("本船航速、SDA、仿真步长必须大于 0。")
            specs = self._current_target_specs(count)
            preset_key = self.PRESET_LABELS.get(self.preset_var.get(), "custom")
            note = f"预设：{self.preset_var.get()}；目标船按表格给定航向航速保向保速。"
            own, targets, _note = build_scenario("head_on", num_targets=count, custom_targets=specs,
                                                 own_speed_kn=own_speed, own_course_deg=own_course)
            env_name = self.environment_var.get()
            try:
                failure_step = max(0, int(self.failure_step_var.get()))
            except ValueError as exc:
                raise ValueError("故障触发步数必须是非负整数。") from exc
            cfg = SimConfig(dt_s=dt_s, sda_nm=sda_nm, max_steps=1800,
                            require_route_recovery=True, random_seed=7)
            obstacles = []
            h = math.radians(own_course)
            if env_name == "风浪流扰动":
                cfg.current_speed_kn, cfg.current_dir_deg = 1.5, 45.0
                cfg.wind_speed_kn, cfg.wind_from_deg = 20.0, 270.0
                cfg.wave_height_m, cfg.wave_period_s = 2.0, 7.5
            elif env_name == "受限能见度":
                cfg.visibility_m = 1000.0
                cfg.sensor_position_sigma_m, cfg.sensor_course_sigma_deg = 12.0, 3.5
            elif env_name == "强传感器噪声":
                cfg.sensor_position_sigma_m, cfg.sensor_course_sigma_deg, cfg.sensor_speed_sigma_kn = 25.0, 5.0, 1.0
                cfg.sensor_dropout_probability = 0.12
            elif env_name in {"GNSS 失效", "罗经偏差", "推进失效", "舵机卡滞", "传感器掉线"}:
                cfg.failure_at_step = failure_step
                cfg.failure_mode = {"GNSS 失效": "gnss_loss", "罗经偏差": "compass_bias",
                    "推进失效": "propulsion_loss", "舵机卡滞": "rudder_jam", "传感器掉线": "sensor_dropout"}[env_name]
            elif env_name == "静态障碍物":
                ahead = 1800.0
                obstacles = [StaticObstacle("OB1", own.east_m + math.sin(h)*ahead,
                    own.north_m + math.cos(h)*ahead, radius_m=100.0)]
            sim = USVSimulator(own, targets, cfg, note + f" 工况：{env_name}。", obstacles=obstacles)
            sim._last_along_track = 0.0
            # Manual reset builds from editable form fields; it is no longer an exact CSV replay.
            self.current_test_case = None
            self.sim = sim
            self._reset_timeline_history()
            self.last_alert_score = 0
            self.logger = RunLogger(APP_DIR / "logs")
            self.sim_count += 1
            self.status_var.set(f"场景已重置。1V{count}；SDA={sda_nm:.2f} NM；每步 {dt_s:g}s；日志：{self.logger.path.name}")
            self.paint()
            self.update_info()
        except Exception as exc:
            messagebox.showerror("参数错误", str(exc))

    def choose_model(self):
        path = filedialog.askopenfilename(title="选择 PPO 模型", initialdir=str(APP_DIR / "models"),
                                          filetypes=[("PPO 模型", "*.zip"), ("所有文件", "*.*")])
        if path:
            self.model_path_var.set(path)

    def load_ppo(self):
        path = Path(self.model_path_var.get())
        if not path.is_absolute():
            path = APP_DIR / path
        try:
            from stable_baselines3 import PPO
            self.ppo_model = PPO.load(str(path))
            self.ppo_path = str(path)
            messagebox.showinfo("模型加载成功", f"已加载模型：\n{path}\n\n请注意：模型必须由本工程的 37 维观测/2 维动作环境训练。")
        except ImportError:
            messagebox.showerror("缺少依赖", "尚未安装强化学习依赖。请在工程目录运行：\npython -m pip install -r requirements.txt")
        except Exception as exc:
            messagebox.showerror("模型加载失败", f"{exc}\n请确认模型文件存在且与当前工程环境兼容。")

    def _action(self):
        if self.sim is None:
            return 0.0, 1.0
        if self.sim.config.require_route_recovery and self.sim.route_recovery_active:
            # Once targets are safely clear, use a dedicated track-recovery action
            # until ownship has returned to its planned route, heading and speed.
            proposed = self.sim.route_recovery_action()
        elif self.policy_var.get().startswith("PPO"):
            if self.ppo_model is None:
                self.load_ppo()
            if self.ppo_model is None:
                raise RuntimeError("尚未加载可用 PPO 模型。")
            import numpy as np
            raw, _ = self.ppo_model.predict(np.asarray(self.sim.observation_values(), dtype=np.float32), deterministic=True)
            raw = raw.tolist()
            proposed = (float(raw[0]) * self.sim.config.max_course_offset_deg,
                        0.80 + 0.20 * max(-1.0, min(1.0, float(raw[1]))))
        else:
            proposed = choose_rule_action(self.sim)
        from usv_colav.safety_filter import shield_action
        d, r, overridden, note = shield_action(self.sim, proposed)
        self.sim.last_safety_filter = note
        return d, r

    _TIMELINE_STATE_FIELDS = (
        "step_count", "elapsed_s", "last_course_delta_deg", "last_speed_ratio",
        "actual_yaw_rate_deg_s", "last_safety_filter", "min_distance_by_target",
        "min_cpa_by_target", "min_obstacle_clearance_m", "sda_violations",
        "obstacle_contacts", "collision", "completed", "terminal_reason",
        "active_failure", "_last_along_track", "_last_sensor_own",
        "_last_sensor_targets", "sensor_snapshot", "route_recovery_active",
        "route_recovery_streak",
    )

    @staticmethod
    def _vessel_timeline_state(vessel: Vessel) -> dict:
        return {key: getattr(vessel, key) for key in ("east_m", "north_m", "speed_kn", "course_deg")}

    def _capture_timeline_frame(self) -> dict | None:
        """Capture one compact, resumable simulation state for the timeline slider."""
        if self.sim is None:
            return None
        sim = self.sim
        state = {name: copy.deepcopy(getattr(sim, name)) for name in self._TIMELINE_STATE_FIELDS}
        return {
            "state": state,
            "own": self._vessel_timeline_state(sim.own),
            "targets": [self._vessel_timeline_state(t) for t in sim.targets],
            "own_position": sim.own.position,
            "target_positions": [t.position for t in sim.targets],
            "rng_state": sim.rng.getstate(),
        }

    def _reset_timeline_history(self):
        self.timeline_history = []
        self.timeline_cursor = 0
        frame = self._capture_timeline_frame()
        if frame is not None:
            self.timeline_history.append(frame)
        self._update_timeline_controls()

    def _append_timeline_frame(self):
        """Save the state after each real simulation step (including hidden fast-forward steps)."""
        if self.sim is None:
            return
        # New simulation steps normally append at the tail. If a future change adds
        # branching from a historical frame, discard that future before appending.
        if self.timeline_cursor < len(self.timeline_history) - 1:
            self.timeline_history = self.timeline_history[:self.timeline_cursor + 1]
        frame = self._capture_timeline_frame()
        if frame is not None:
            self.timeline_history.append(frame)
            self.timeline_cursor = len(self.timeline_history) - 1

    def _restore_timeline_frame(self, index: int, update_controls: bool = True):
        if self.sim is None or not self.timeline_history:
            return
        index = max(0, min(int(index), len(self.timeline_history) - 1))
        frame = self.timeline_history[index]
        sim = self.sim
        for name, value in frame["state"].items():
            setattr(sim, name, copy.deepcopy(value))
        for name, value in frame["own"].items():
            setattr(sim.own, name, value)
        for vessel, vessel_state in zip(sim.targets, frame["targets"]):
            for name, value in vessel_state.items():
                setattr(vessel, name, value)
        # Rebuild the visible trails from the history's position stream. This lets
        # the user scrub backward and forward without losing later frames.
        prefix = self.timeline_history[:index + 1]
        sim.own.track = [item["own_position"] for item in prefix]
        for target_index, target in enumerate(sim.targets):
            target.track = [item["target_positions"][target_index] for item in prefix]
        sim.rng.setstate(frame["rng_state"])
        self.timeline_cursor = index
        if update_controls:
            self._update_timeline_controls()

    def _update_timeline_controls(self):
        if not hasattr(self, "timeline_scale"):
            return
        total_frames = max(0, len(self.timeline_history) - 1)
        self._updating_timeline = True
        try:
            self.timeline_scale.configure(to=total_frames)
            self.timeline_index_var.set(float(min(self.timeline_cursor, total_frames)))
            current_t = self.timeline_history[self.timeline_cursor]["state"]["elapsed_s"] if self.timeline_history else 0.0
            end_t = self.timeline_history[-1]["state"]["elapsed_s"] if self.timeline_history else 0.0
            current_step = self.timeline_history[self.timeline_cursor]["state"]["step_count"] if self.timeline_history else 0
            end_step = self.timeline_history[-1]["state"]["step_count"] if self.timeline_history else 0
            self.timeline_label_var.set(f"t={current_t:.0f}/{end_t:.0f}s  step={current_step}/{end_step}")
        finally:
            self._updating_timeline = False

    def _cancel_animation_callback(self):
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except Exception:
                pass
            self.after_id = None

    def _on_timeline_drag(self, value):
        if self._updating_timeline or not self.timeline_history:
            return
        if self.running:
            self.running = False
            self._cancel_animation_callback()
            self.start_btn.configure(text="继续运行")
        index = max(0, min(int(round(float(value))), len(self.timeline_history) - 1))
        frame = self.timeline_history[index]
        t = frame["state"]["elapsed_s"]
        step = frame["state"]["step_count"]
        end_t = self.timeline_history[-1]["state"]["elapsed_s"]
        end_step = self.timeline_history[-1]["state"]["step_count"]
        self.timeline_label_var.set(f"预览 t={t:.0f}/{end_t:.0f}s  step={step}/{end_step}")

    def _seek_timeline(self, _event=None):
        if self._updating_timeline or not self.timeline_history or self.sim is None:
            return
        if self.running:
            self.running = False
            self._cancel_animation_callback()
            self.start_btn.configure(text="继续运行")
        index = max(0, min(int(round(self.timeline_index_var.get())), len(self.timeline_history) - 1))
        self._restore_timeline_frame(index)
        self.paint()
        self.update_info()
        if self.sim.completed:
            self.start_btn.configure(text="场景已结束")
        else:
            self.start_btn.configure(text="继续运行" if index > 0 else "开始运行")
        self.status_var.set(
            f"时间进度回看：step={self.sim.step_count}，t={self.sim.elapsed_s:.0f}s；"
            "可继续拖动进度条，或点击继续运行沿时间线播放。"
        )

    def _advance_steps(self, count: int = 1, render: bool = True):
        """Advance simulator/controller steps and optionally render the resulting state.

        Animation playback always calls this with count=1, so each step is a visible
        frame. The time-skip command may pass a larger count intentionally; that is an
        explicit skip operation and renders only its final state.
        """
        if self.sim is None:
            self.reset_simulation()
        if self.sim is None or self.sim.completed:
            return
        last_action = (0.0, 1.0)
        last_reason = "运行中"
        steps_done = 0
        replayed_history = False
        try:
            for _ in range(max(1, int(count))):
                if self.sim is None or self.sim.completed:
                    break
                # If the timeline cursor is behind the newest saved frame, advance
                # through recorded states first. This provides smooth playback after
                # the user scrubs to any earlier moment, without re-running the policy.
                if self.timeline_cursor < len(self.timeline_history) - 1:
                    self.timeline_cursor += 1
                    self._restore_timeline_frame(self.timeline_cursor, update_controls=False)
                    replayed_history = True
                    steps_done += 1
                    last_reason = "回放历史态势"
                    if self.sim.completed:
                        self.running = False
                        self.start_btn.configure(text="场景已结束")
                        break
                    continue
                last_action = self._action()
                _reward, done, last_reason = self.sim.step(last_action)
                if self.logger:
                    self.logger.write(self.sim)
                self._append_timeline_frame()
                steps_done += 1
                if done:
                    self.running = False
                    self.start_btn.configure(text="场景已结束")
                    break
            if self.sim is None:
                return
            self._update_timeline_controls()
            shield_text = getattr(self.sim, "last_safety_filter", "")
            suffix = f" | 安全层：{shield_text}" if shield_text else ""
            if replayed_history:
                self.status_var.set(
                    f"历史态势回放：step={self.sim.step_count}  t={self.sim.elapsed_s:.0f}s  "
                    f"本次显示 {steps_done} 帧  {self.sim.terminal_reason}"
                )
            else:
                route_text = "；本船正在恢复计划航线" if getattr(self.sim, "route_recovery_active", False) else ""
                self.status_var.set(
                    f"step={self.sim.step_count}  t={self.sim.elapsed_s:.0f}s  "
                    f"本次推进 {steps_done} 步  指令偏角={last_action[0]:+.1f}°  "
                    f"速度比例={last_action[1]:.2f}  {last_reason}{route_text}{suffix}"
                )
            if self.sim.completed:
                self.status_var.set(self.status_var.get() + f"；仿真结束：{self.sim.terminal_reason}")
            if render:
                self.paint()
                self.update_info()
        except Exception as exc:
            self.running = False
            self.start_btn.configure(text="开始运行")
            messagebox.showerror("仿真运行错误", str(exc))

    def step_once(self):
        self._advance_steps(1, render=True)

    def toggle_running(self):
        if self.sim is None:
            self.reset_simulation()
        if self.sim is None or self.sim.completed:
            return
        if self.running:
            # Cancel the queued tick so pause/resume cannot create duplicate loops.
            self.running = False
            if self.after_id is not None:
                try:
                    self.root.after_cancel(self.after_id)
                except Exception:
                    pass
                self.after_id = None
            self.start_btn.configure(text="继续运行")
            self.status_var.set("仿真已暂停；可调整倍速后继续运行。")
            return
        self.running = True
        self.start_btn.configure(text="暂停运行")
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except Exception:
                pass
            self.after_id = None
        self._animation_tick()

    def _animation_tick(self):
        # Consume the currently queued callback. One simulator step equals one frame;
        # never batch multiple steps in accelerated or unlimited playback.
        self.after_id = None
        if not self.running:
            return
        steps_per_frame, delay_ms = playback_tick_plan(self.playback_speed_var.get())
        self._advance_steps(steps_per_frame, render=True)
        if self.running:
            if delay_ms == 0:
                # Yield to Tk's pending canvas redraw tasks between steps, with no
                # deliberate wait inserted in unlimited mode.
                self.after_id = self.root.after_idle(self._animation_tick)
            else:
                # High speed means a shorter interval between rendered frames, not
                # more simulator steps hidden between two frames.
                self.after_id = self.root.after(delay_ms, self._animation_tick)

    def _fast_forward_simulated_time(self):
        selected = self._selected_test_case()
        if self.sim is None or self.current_test_case is None:
            messagebox.showwarning("尚未加载场景", "请先在‘测试用例回放’页选择并加载一个场景。")
            return
        loaded_group, loaded_path = self.current_test_case
        if selected is None or selected[0] != loaded_group or selected[1].name != loaded_path.name:
            messagebox.showwarning("用例尚未加载", "当前列表选择的用例与正在运行的仿真不是同一个。请先点击‘加载 / 重新开始’，再快进。")
            return
        if self.sim.completed:
            messagebox.showinfo("仿真已结束", "当前场景已经结束。请重新加载用例后再快进。")
            return
        try:
            seconds = float(self.case_skip_seconds_var.get())
            if not math.isfinite(seconds) or seconds <= 0:
                raise ValueError
        except (TypeError, ValueError):
            messagebox.showerror("输入错误", "快进时长必须是大于 0 的秒数，例如 1800。")
            return
        self.running = False
        self.start_btn.configure(text="开始运行")
        dt_s = self.sim.config.dt_s
        requested_steps = max(1, int(math.ceil(seconds / dt_s)))
        remaining_steps = max(0, self.sim.config.max_steps - self.sim.step_count)
        actual_steps = min(requested_steps, remaining_steps)
        if actual_steps <= 0:
            messagebox.showinfo("达到仿真上限", "当前场景已到达最大仿真步数，不能继续快进。")
            return
        start_time = self.sim.elapsed_s
        self.status_var.set(f"正在按控制策略快进：目标 {seconds:.1f}s，约 {actual_steps} 步……")
        self.root.update_idletasks()
        self._advance_steps(actual_steps, render=True)
        actual_advanced = self.sim.elapsed_s - start_time if self.sim else 0.0
        suffix = "（已触发场景终止条件）" if self.sim and self.sim.completed else ""
        self.status_var.set(
            f"快进完成：仿真时间推进 {actual_advanced:.1f}s（请求 {seconds:.1f}s，步长 {dt_s:g}s）; "
            f"当前 t={self.sim.elapsed_s:.1f}s，step={self.sim.step_count}{suffix}"
        )

    def stop_and_save(self):
        self.running = False
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except Exception:
                pass
            self.after_id = None
        self.start_btn.configure(text="开始运行")
        if self.logger:
            p = self.logger.path
            self.logger.close(); self.logger = None
            self.status_var.set(f"仿真已停止，数据已保存：{p}")
            messagebox.showinfo("日志保存完成", f"逐步态势、CPA/TCPA、风险等级及船舶状态已保存到：\n{p}")
        else:
            self.status_var.set("仿真已停止；当前没有打开的日志。")

    def update_info(self):
        if self.sim is None:
            return
        sim = self.sim
        metrics = sim.metrics()
        severity_order = {"无危险": 0, "碰撞危险": 1, "紧迫局面": 2}
        severity = max((severity_order.get(m["risk"], 0) for m in metrics), default=0)
        if severity > self.last_alert_score:
            try:
                self.root.bell()  # visual label + system bell for a newly escalated risk level
            except Exception:
                pass
        self.last_alert_score = severity
        min_dist = min((m["distance_m"] for m in metrics), default=math.inf)
        own_text = (
            f"时间：{sim.elapsed_s:.1f} s  |  步数：{sim.step_count}\n"
            f"位置：E={sim.own.east_m:.1f} m，N={sim.own.north_m:.1f} m\n"
            f"航速：{sim.own.speed_kn:.2f} kn  |  航向：{sim.own.course_deg:.1f}°\n"
            f"计划航向：{sim.own.planned_course_deg:.1f}°  |  横向偏移：{sim._cross_track_m():.1f} m\n"
            f"航行阶段：{'避碰后恢复计划航线' if getattr(sim, 'route_recovery_active', False) else '会遇监视 / 避碰中'}  |  "
            f"航线恢复稳定计数：{getattr(sim, 'route_recovery_streak', 0)}\n"
            f"本回合最小距离：{m_to_nm(min_dist):.3f} NM  |  SDA：{sim.config.sda_nm:.3f} NM\n"
            f"历史 SDA 违规目标：{', '.join(sorted(sim.sda_violations)) or '无'}\n"
            f"传感器可见目标：{sim.sensor_snapshot.get('visible_targets', 0)}/{len(sim.targets)}；故障：{sim.active_failure}\n"
            f"障碍物接触：{', '.join(sorted(sim.obstacle_contacts)) or '无'}\n"
            f"碰撞标记：{'是' if sim.collision else '否'}  |  状态：{sim.terminal_reason}"
        )
        self.own_info.configure(state="normal"); self.own_info.delete("1.0", "end"); self.own_info.insert("1.0", own_text); self.own_info.configure(state="disabled")
        blocks = []
        for m in metrics:
            tcpa = m["tcpa_min"]
            tcpa_s = "∞" if math.isinf(tcpa) else f"{tcpa:.2f} min"
            blocks.append(
                f"{m['target']}\n"
                f"  距离={m['distance_nm']:.3f} NM | 方位={m['relative_bearing_deg']:+.1f}°\n"
                f"  CPA={m['cpa_nm']:.3f} NM | TCPA={tcpa_s} | SDA={m['sda_nm']:.3f} NM\n"
                f"  局面={m['encounter']} | 本船让路建议={('是' if m['own_give_way'] else '保持警戒')} | 风险={m['risk']}"
            )
        self.target_info.configure(state="normal"); self.target_info.delete("1.0", "end"); self.target_info.insert("1.0", "\n\n".join(blocks)); self.target_info.configure(state="disabled")

    def paint(self):
        if self.sim is None:
            return
        c = self.canvas
        c.delete("all")
        w = max(500, c.winfo_width()); h = max(420, c.winfo_height())
        sim = self.sim
        points = [sim.own.position] + [p for p in sim.own.track[::max(1, len(sim.own.track)//500)]]
        for t in sim.targets:
            points += [t.position] + t.track[::max(1, len(t.track)//500)]
        for obstacle in sim.obstacles:
            points.append(obstacle.position)
        # Include forward projections and route in the view bounds.
        route_dx = math.sin(math.radians(float(sim.own.planned_course_deg))) * NM_TO_M * 8
        route_dy = math.cos(math.radians(float(sim.own.planned_course_deg))) * NM_TO_M * 8
        points += [(sim.own.east_m + route_dx, sim.own.north_m + route_dy)]
        for t in sim.targets:
            from usv_colav.geometry import velocity_from_course
            vx, vy = velocity_from_course(t.speed_kn, t.course_deg)
            points.append((t.east_m + vx * 600, t.north_m + vy * 600))
        xs = [p[0] for p in points]; ys = [p[1] for p in points]
        xmin, xmax = min(xs), max(xs); ymin, ymax = min(ys), max(ys)
        pad = 0.10
        xr = max(1000.0, xmax - xmin); yr = max(1000.0, ymax - ymin)
        xmin -= xr*pad; xmax += xr*pad; ymin -= yr*pad; ymax += yr*pad
        scale = min((w-70)/(xmax-xmin), (h-70)/(ymax-ymin))
        def xy(p):
            return 35 + (p[0]-xmin)*scale, h-35 - (p[1]-ymin)*scale
        # Background grid: approx one nautical mile.
        xstart = math.floor(xmin/NM_TO_M)*NM_TO_M
        while xstart <= xmax:
            sx, _ = xy((xstart, ymin)); c.create_line(sx, 25, sx, h-25, fill="#e2e8f0", dash=(2, 5))
            c.create_text(sx+3, h-14, text=f"E {xstart/NM_TO_M:.0f} NM", fill="#64748b", anchor="w", font=("Arial", 8))
            xstart += NM_TO_M
        ystart = math.floor(ymin/NM_TO_M)*NM_TO_M
        while ystart <= ymax:
            _, sy = xy((xmin, ystart)); c.create_line(25, sy, w-25, sy, fill="#e2e8f0", dash=(2, 5))
            c.create_text(30, sy-4, text=f"N {ystart/NM_TO_M:.0f} NM", fill="#64748b", anchor="w", font=("Arial", 8))
            ystart += NM_TO_M
        # Original planned route.
        own_start = (0.0, 0.0); own_future = (route_dx, route_dy)
        x1,y1=xy(own_start); x2,y2=xy(own_future)
        c.create_line(x1,y1,x2,y2,fill="#94a3b8",width=2,dash=(8,5))
        c.create_text(x2, y2, text="计划航线", fill="#64748b", anchor="nw")
        # Static obstacles are drawn with their safety radii.
        for obstacle in sim.obstacles:
            ox, oy = xy(obstacle.position)
            rr = max(4, obstacle.radius_m * scale)
            c.create_oval(ox-rr, oy-rr, ox+rr, oy+rr, fill="#64748b", outline="#334155", width=2)
            c.create_text(ox, oy, text=obstacle.name, fill="#ffffff", font=("Arial", 8, "bold"))
        # Tracks and projected target courses.
        for idx, t in enumerate(sim.targets):
            col = ["#dc2626", "#d97706", "#7c3aed"][idx % 3]
            if len(t.track) > 1:
                coords = [xy(p) for p in t.track[::max(1, len(t.track)//500)]]
                c.create_line(*[v for p in coords for v in p], fill=col, width=2)
            from usv_colav.geometry import predicted_position
            projection = [t.position, predicted_position(t.position, t.speed_kn, t.course_deg, 600)]
            p1=xy(projection[0]); p2=xy(projection[1])
            c.create_line(*p1,*p2,fill=col,width=1,dash=(3,4))
            self._draw_boat(t.position, t.course_deg, col, t.name, xy, c, own=False)
        if len(sim.own.track) > 1:
            coords = [xy(p) for p in sim.own.track[::max(1, len(sim.own.track)//700)]]
            c.create_line(*[v for p in coords for v in p], fill="#0369a1", width=3)
        self._draw_boat(sim.own.position, sim.own.course_deg, "#0369a1", "OV", xy, c, own=True)
        severity_order = {"无危险": 0, "碰撞危险": 1, "紧迫局面": 2}
        current_risk = max((severity_order.get(m["risk"], 0) for m in sim.metrics()), default=0)
        risk_label = ["无危险", "碰撞危险", "紧迫局面"][current_risk]
        risk_color = ["#166534", "#b45309", "#b91c1c"][current_risk]
        case_suffix = f"  |  用例={self.current_test_case[0]}/{self.current_test_case[1].stem}" if self.current_test_case else ""
        c.create_text(14, 10, text=f"1V{len(sim.targets)}  |  t={sim.elapsed_s:.0f}s  |  策略={self.policy_var.get()}  |  当前风险={risk_label}{case_suffix}",
                      anchor="nw", fill=risk_color, font=("Microsoft YaHei UI", 10, "bold"))
        # Status banner in lower-left corner.
        c.create_rectangle(12, h-47, min(w-12, 470), h-12, fill="#ffffff", outline="#cbd5e1")
        c.create_text(20, h-30, text=f"min separation={m_to_nm(min((m['distance_m'] for m in sim.metrics()), default=0)):.3f} NM | SDA violations={len(sim.sda_violations)}",
                      anchor="w", fill="#0f172a", font=("Arial", 10))

        # Flush the completed frame only after all canvas items are drawn. This avoids
        # the old clear-screen flash and allows each simulation step to be painted.
        try:
            c.update_idletasks()
        except tk.TclError:
            pass

    def _draw_boat(self, pos, course, color, label, xy, canvas, own=False):
        x, y = xy(pos)
        size = 12 if own else 9
        # Course axis in screen coordinates: east = sin(course), screen-down = -cos(course).
        a = math.radians(course)
        fx, fy = math.sin(a), -math.cos(a)
        rx, ry = math.cos(a), math.sin(a)
        nose = (x + fx*size*1.6, y + fy*size*1.6)
        left = (x - fx*size*0.8 + rx*size*0.65, y - fy*size*0.8 - ry*size*0.65)
        right = (x - fx*size*0.8 - rx*size*0.65, y - fy*size*0.8 + ry*size*0.65)
        canvas.create_polygon(*nose, *left, *right, fill=color, outline="#0f172a", width=1)
        canvas.create_text(x+size+4, y-size-4, text=label, fill=color, anchor="sw", font=("Arial", 10, "bold"))

    def close_app(self):
        self.running = False
        self._cancel_animation_callback()
        if self.logger:
            self.logger.close(); self.logger = None
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        style = ttk.Style()
        if "vista" in style.theme_names():
            style.theme_use("vista")
    except Exception:
        pass
    USVColavApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
