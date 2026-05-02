import tkinter as tk
from tkinter import messagebox
import math

PRESSURE_ANGLE_RAD   = math.radians(20.0)
DEDENDUM_COEFFICIENT = 1.25
EXPORT_PRECISION     = 3
BACKLASH_FACTOR      = 0.05  # BACKLASH = module * BACKLASH_FACTOR


def calc_gear_params(num_teeth: int, module: float, backlash_factor: float = BACKLASH_FACTOR) -> dict:
    backlash        = module * backlash_factor
    pitch_radius    = module * num_teeth / 2
    addendum_radius = pitch_radius + module
    base_radius     = pitch_radius * math.cos(PRESSURE_ANGLE_RAD)
    dedendum_radius = pitch_radius - DEDENDUM_COEFFICIENT * module
    t_tip           = math.sqrt((addendum_radius / base_radius)**2 - 1)
    t_pitch         = math.sqrt((pitch_radius    / base_radius)**2 - 1)
    inv_pitch       = t_pitch - math.atan(t_pitch)
    offset_angle    = math.pi / (2 * num_teeth) - inv_pitch + backlash / (2 * pitch_radius)
    tau             = 2 * math.pi / num_teeth
    px_tip          = base_radius * (math.cos(t_tip) + t_tip * math.sin(t_tip))
    py_tip          = base_radius * (math.sin(t_tip) - t_tip * math.cos(t_tip))
    tip_angle       = math.atan2(py_tip, px_tip)
    # ピッチ点でのインボリュート角（噛み合い位相計算用）
    inv_pitch_angle = math.atan2(
        base_radius * (math.sin(t_pitch) - t_pitch * math.cos(t_pitch)),
        base_radius * (math.cos(t_pitch) + t_pitch * math.sin(t_pitch)),
    )
    return dict(
        num_teeth        = num_teeth,
        module           = module,
        backlash         = backlash,
        pitch_radius     = pitch_radius,
        addendum_radius  = addendum_radius,
        base_radius      = base_radius,
        dedendum_radius  = dedendum_radius,
        t_tip            = t_tip,
        offset_angle     = offset_angle,
        tip_angle        = tip_angle,
        inv_pitch_angle  = inv_pitch_angle,
        tau              = tau,
    )


def _pt(px, py, p):
    """座標を出力文字列に変換"""
    return f"{0:.{p}f} {px:.{p}f} {py:.{p}f}\n"


def export_dsm_polyline(params: dict, filename: str = "gear_dsm.txt"):
    """
    DSM用スプラインファイルを出力する。

    ヘッダ:
        3d=true
        Polyline=false   … スプライン曲線
        Fit=false        … 指定点を必ず通る
        fittol=1.0e-3    … 座標丸め解像度

    出力構造（歯数を z とする）:
        (1) 歯底弧          × z    （弧間は改行）
        (2) 根元直線        × z×2  （直線間は改行）
        (3) 右インボリュート × z    （曲線間は改行）
        (4) 左インボリュート × z    （曲線間は改行）
        (5) 歯先弧          × z    （弧間は改行）

    fittol=1.0e-3 の精度により隣接セクションの端点は自動接続される。
    """
    p     = EXPORT_PRECISION
    rb    = params["base_radius"]
    rf    = params["dedendum_radius"]
    ra    = params["addendum_radius"]
    oa    = params["offset_angle"]
    ta    = params["tip_angle"]
    tau   = params["tau"]
    z     = params["num_teeth"]
    t_tip = params["t_tip"]
    n_arc = 20
    n_inv = 50

    def involute_pt(t, angle, flip=1):
        px = rb * (math.cos(t) + t * math.sin(t))
        py = rb * (math.sin(t) - t * math.cos(t)) * flip
        rx = px * math.cos(angle) - py * math.sin(angle)
        ry = px * math.sin(angle) + py * math.cos(angle)
        return rx, ry

    def arc_pt(r, angle):
        return r * math.cos(angle), r * math.sin(angle)

    teeth = []
    for i in range(z):
        base = i * tau
        teeth.append(dict(
            arc_ded_start = (i - 1) * tau + tau - oa,
            arc_ded_end   = base + oa,
            arc_add_start = base + oa + ta,
            arc_add_end   = base + tau - oa - ta,
            a_right       = base + oa,
            a_left        = base + tau - oa,
        ))

    with open(filename, "w") as f:
        f.write("3d=true\n")
        f.write("Polyline=false\n")
        f.write("Fit=false\n")
        f.write("fittol=1.0e-3\n")
        f.write("\n")

        # (1) 歯底弧 × z
        for i, t in enumerate(teeth):
            for k in range(n_arc + 1):
                a = t["arc_ded_start"] + (t["arc_ded_end"] - t["arc_ded_start"]) * k / n_arc
                f.write(_pt(*arc_pt(rf, a % (2*math.pi)), p))
            if i < z - 1:
                f.write("\n")
        f.write("\n")

        # (2) 根元直線 × z×2
        segs = []
        for t in teeth:
            segs.append((arc_pt(rf, t["arc_ded_end"] % (2*math.pi)), arc_pt(rb, t["a_right"])))
            segs.append((arc_pt(rb, t["a_left"]), arc_pt(rf, t["a_left"] % (2*math.pi))))
        for i, (start, end) in enumerate(segs):
            f.write(_pt(*start, p))
            f.write(_pt(*end,   p))
            if i < len(segs) - 1:
                f.write("\n")
        f.write("\n")

        # (3) 右インボリュート × z  （基礎円→歯先）
        for i, t in enumerate(teeth):
            for k in range(n_inv + 1):
                f.write(_pt(*involute_pt(t_tip * k / n_inv, t["a_right"], 1), p))
            if i < z - 1:
                f.write("\n")
        f.write("\n")

        # (4) 左インボリュート × z  （歯先→基礎円）
        for i, t in enumerate(teeth):
            for k in range(n_inv + 1):
                f.write(_pt(*involute_pt(t_tip * (n_inv - k) / n_inv, t["a_left"], -1), p))
            if i < z - 1:
                f.write("\n")
        f.write("\n")

        # (5) 歯先弧 × z
        for i, t in enumerate(teeth):
            for k in range(n_arc + 1):
                a = t["arc_add_start"] + (t["arc_add_end"] - t["arc_add_start"]) * k / n_arc
                f.write(_pt(*arc_pt(ra, a % (2*math.pi)), p))
            if i < z - 1:
                f.write("\n")
        f.write("\n")

        # 中心マーカー: z方向 ±1mm の線（軸穴位置の基準点）
        # DSMの座標順は z x y なので: z=±1, x=0, y=0
        f.write(f" 1.000  0.000  0.000\n")
        f.write(f"-1.000  0.000  0.000\n")

    print(f"出力完了: {filename}")


def export_log(p1: dict, p2: dict, filename1: str, filename2: str):
    """歯車パラメータのログファイルを出力する。"""
    import datetime
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # 噛み合い圧力角（バックラッシュ補正後）
    def mesh_pressure_angle(pa, pa2):
        try:
            from scipy.optimize import brentq
            r1, r2   = pa["pitch_radius"],  pa2["pitch_radius"]
            rb1, rb2 = pa["base_radius"],   pa2["base_radius"]
            z1, z2   = pa["num_teeth"],     pa2["num_teeth"]
            m        = pa["module"]
            bl       = pa["backlash"]
            s1 = 2*r1*(math.pi/(2*z1) - (math.sqrt((r1/rb1)**2-1) - math.atan(math.sqrt((r1/rb1)**2-1)))) - bl
            s2 = 2*r2*(math.pi/(2*z2) - (math.sqrt((r2/rb2)**2-1) - math.atan(math.sqrt((r2/rb2)**2-1)))) - bl
            inv_a  = math.tan(PRESSURE_ANGLE_RAD) - PRESSURE_ANGLE_RAD
            inv_aw = (s1+s2)/(r1+r2) - math.pi/(z1+z2) + inv_a
            aw = brentq(lambda a: math.tan(a)-a-inv_aw, 0.001, math.pi/3)
            return math.degrees(aw)
        except Exception:
            return None

    aw = mesh_pressure_angle(p1, p2)
    center_dist = p1["pitch_radius"] + p2["pitch_radius"]
    gear_ratio  = p2["num_teeth"] / p1["num_teeth"]

    def write_log(f, p, partner, gear_no):
        m   = p["module"]
        z   = p["num_teeth"]
        bl  = p["backlash"]
        r   = p["pitch_radius"]
        ra  = p["addendum_radius"]
        rf  = p["dedendum_radius"]
        rb  = p["base_radius"]
        ha  = m
        hf  = 1.25 * m
        h   = ha + hf
        cp  = math.pi * m
        s   = 2 * r * (math.pi/(2*z) - (math.sqrt((r/rb)**2-1) - math.atan(math.sqrt((r/rb)**2-1)))) - bl

        f.write(f"{'='*60}\n")
        f.write(f"  歯車{gear_no} (Gear {gear_no}) パラメータログ\n")
        f.write(f"{'='*60}\n")
        f.write(f"  出力日時 (Timestamp)          : {now}\n")
        f.write(f"  出力ファイル (Output file)    : gear{gear_no}_dsm.txt\n")
        f.write(f"  座標精度 (Export precision)   : {EXPORT_PRECISION} 桁\n")
        f.write(f"  fittol                        : 1.0e-3\n")
        f.write(f"\n")
        f.write(f"  ── 基本パラメータ (Basic parameters) ──\n")
        f.write(f"  歯数 (Teeth)                  : {z}\n")
        f.write(f"  モジュール (Module)            : {m:.3f} [mm]\n")
        f.write(f"  圧力角 (Pressure angle)        : {math.degrees(PRESSURE_ANGLE_RAD):.1f} [deg]\n")
        f.write(f"  歯底係数 (Dedendum coeff)      : {DEDENDUM_COEFFICIENT:.2f}\n")
        f.write(f"  バックラッシュ係数 (BL factor) : {bl/m:.4f}\n")
        f.write(f"  バックラッシュ (Backlash)      : {bl:.4f} [mm]\n")
        f.write(f"\n")
        f.write(f"  ── 寸法 (Dimensions) ──\n")
        f.write(f"  ピッチ円直径 (Pitch dia)       : {r*2:.4f} [mm]\n")
        f.write(f"  歯先円直径 (Addendum dia)      : {ra*2:.4f} [mm]\n")
        f.write(f"  歯底円直径 (Dedendum dia)      : {rf*2:.4f} [mm]\n")
        f.write(f"  基礎円直径 (Base circle dia)   : {rb*2:.4f} [mm]\n")
        f.write(f"  歯先のたけ (Addendum)          : {ha:.4f} [mm]\n")
        f.write(f"  歯元のたけ (Dedendum)          : {hf:.4f} [mm]\n")
        f.write(f"  全歯たけ (Whole depth)         : {h:.4f} [mm]\n")
        f.write(f"  円ピッチ (Circular pitch)      : {cp:.4f} [mm]\n")
        f.write(f"  歯厚 (Tooth thickness)         : {s:.4f} [mm]\n")
        f.write(f"\n")
        f.write(f"  ── 歯車対情報 (Gear pair) ──\n")
        f.write(f"  相手歯数 (Mating teeth)        : {partner['num_teeth']}\n")
        f.write(f"  中心距離 (Center distance)     : {center_dist:.4f} [mm]\n")
        f.write(f"  歯車比 (Gear ratio)            : {gear_ratio:.6f}\n")
        if aw is not None:
            f.write(f"  噛み合い圧力角 (Working PA)    : {aw:.4f} [deg]\n")
        f.write(f"{'='*60}\n")

    with open(filename1, "w", encoding="utf-8") as f:
        write_log(f, p1, p2, 1)
    print(f"出力完了: {filename1}")

    with open(filename2, "w", encoding="utf-8") as f:
        write_log(f, p2, p1, 2)
    print(f"出力完了: {filename2}")


def draw_gear(canvas, params, cx, cy, angle_offset, scale, ox, oy):
    """歯車1枚をキャンバスに描画する。(cx,cy) は歯車中心の論理座標。"""

    def to_screen(px, py):
        return ox + (cx + px) * scale, oy - (cy + py) * scale

    z     = params["num_teeth"]
    oa    = params["offset_angle"]
    ta    = params["tip_angle"]
    tau   = params["tau"]
    rb    = params["base_radius"]
    rf    = params["dedendum_radius"]
    ra    = params["addendum_radius"]
    rp    = params["pitch_radius"]
    t_tip = params["t_tip"]
    n     = 20
    n_inv = 50

    def involute_pt(t, angle, flip=1):
        px = rb * (math.cos(t) + t * math.sin(t))
        py = rb * (math.sin(t) - t * math.cos(t)) * flip
        rx = px * math.cos(angle) - py * math.sin(angle)
        ry = px * math.sin(angle) + py * math.cos(angle)
        return rx, ry

    # 参照円
    def draw_circle(r, color, dash=()):
        pts = [to_screen(r * math.cos(a), r * math.sin(a))
               for a in (2*math.pi * k / 120 for k in range(121))]
        canvas.create_line(pts, fill=color, width=1, dash=dash)

    draw_circle(ra, "#378ADD", (5, 4))
    draw_circle(rp, "#1D9E75", (5, 4))
    draw_circle(rb, "#FAC775", (3, 3))
    draw_circle(rf, "#D85A30")

    for i in range(z):
        base = i * tau + angle_offset

        # 歯底弧
        arc_start = base - oa
        arc_end   = base + oa
        seg = [to_screen(rf * math.cos(arc_start + (arc_end - arc_start) * k / n),
                         rf * math.sin(arc_start + (arc_end - arc_start) * k / n))
               for k in range(n + 1)]
        canvas.create_line(seg, fill="white", width=1.5)

        # 右根元直線
        canvas.create_line(
            *to_screen(rf * math.cos(base + oa), rf * math.sin(base + oa)),
            *to_screen(rb * math.cos(base + oa), rb * math.sin(base + oa)),
            fill="#aaaaaa", width=1.5)

        # 左根元直線
        canvas.create_line(
            *to_screen(rb * math.cos(base + tau - oa), rb * math.sin(base + tau - oa)),
            *to_screen(rf * math.cos(base + tau - oa), rf * math.sin(base + tau - oa)),
            fill="#aaaaaa", width=1.5)

        # 右インボリュート
        seg = [to_screen(*involute_pt(t_tip * k / n_inv, base + oa, 1))
               for k in range(n_inv + 1)]
        canvas.create_line(seg, fill="#9FE1CB", width=1.5)

        # 左インボリュート
        seg = [to_screen(*involute_pt(t_tip * k / n_inv, base + tau - oa, -1))
               for k in range(n_inv + 1)]
        canvas.create_line(seg, fill="#9FE1CB", width=1.5)

        # 歯先弧
        arc_start = base + oa + ta
        arc_end   = base + tau - oa - ta
        seg = [to_screen(ra * math.cos(arc_start + (arc_end - arc_start) * k / n),
                         ra * math.sin(arc_start + (arc_end - arc_start) * k / n))
               for k in range(n + 1)]
        canvas.create_line(seg, fill="#378ADD", width=1.5)

    # 中心点
    cx_s, cy_s = to_screen(0, 0)
    canvas.create_oval(cx_s-3, cy_s-3, cx_s+3, cy_s+3, fill="white")


def draw_scale_bar(canvas, scale, canvas_w, canvas_h):
    """画面中央に円スケールを描画する。円の直径が基準寸法、中央に数値表示。"""
    candidates = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500]
    bar_max_px  = 120

    bar_mm = candidates[0]
    for c in candidates:
        if c * scale <= bar_max_px:
            bar_mm = c
        else:
            break

    r  = bar_mm * scale / 2
    cx = canvas_w / 2
    cy = canvas_h / 2

    canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                       outline="white", width=1.5)
    canvas.create_text(cx, cy, text=f"⌀{bar_mm:g} mm",
                       fill="white", font=("Helvetica", 10), anchor="center")


def draw_legend(canvas):
    items = [
        ("#378ADD", (5, 4), "歯先円 (addendum_radius)"),
        ("#1D9E75", (5, 4), "ピッチ円 (pitch_radius)"),
        ("#FAC775", (3, 3), "基礎円 (base_radius)"),
        ("#D85A30", (),     "歯底円 (dedendum_radius)"),
        ("white",   (),     "歯底弧 (dedendum_arc)"),
        ("#aaaaaa", (),     "根元直線 (root_line)"),
        ("#9FE1CB", (),     "インボリュート (involute)"),
        ("#378ADD", (),     "歯先弧 (addendum_arc)"),
    ]
    x0, y0, dy, llen = 10, 10, 22, 26
    for i, (color, dash, label) in enumerate(items):
        y = y0 + i * dy
        canvas.create_line(x0, y, x0 + llen, y, fill=color, width=2, dash=dash)
        canvas.create_text(x0 + llen + 6, y, text=label, anchor="w",
                           fill="white", font=("Helvetica", 10))


def main():
    ZOOM_FACTOR = 1.15
    CANVAS_W    = 1800
    CANVAS_H    = 900

    root = tk.Tk()
    root.title("Gear Pair Viewer")

    # ── コントロールバー ──────────────────────────
    ctrl = tk.Frame(root, bg="#222222", pady=6)
    ctrl.pack(fill="x")

    def labeled_entry(parent, label, default, width=6):
        tk.Label(parent, text=label, bg="#222222", fg="white",
                 font=("Helvetica", 11)).pack(side="left", padx=(12, 4))
        var = tk.StringVar(value=str(default))
        tk.Entry(parent, textvariable=var, width=width,
                 font=("Helvetica", 11)).pack(side="left")
        return var

    var_z1      = labeled_entry(ctrl, "歯数1",           19)
    var_z2      = labeled_entry(ctrl, "歯数2",           37)
    var_module  = labeled_entry(ctrl, "モジュール",       2.0)
    var_bl      = labeled_entry(ctrl, "バックラッシュ係数", BACKLASH_FACTOR, width=5)

    # 計算結果表示ラベル
    def info_label(parent, text="―"):
        tk.Label(parent, text=text, bg="#222222", fg="#aaaaaa",
                 font=("Helvetica", 11)).pack(side="left", padx=(4, 0))
        lbl = tk.Label(parent, text="―", bg="#222222", fg="white",
                       font=("Helvetica", 11, "bold"))
        lbl.pack(side="left", padx=(2, 12))
        return lbl

    lbl_da1 = info_label(ctrl, "歯先円1:")
    lbl_da2 = info_label(ctrl, "歯先円2:")
    lbl_cd  = info_label(ctrl, "中心距離:")

    state = {"p1": None, "p2": None,
             "scale": 1.0, "ox": CANVAS_W/2, "oy": CANVAS_H/2}

    def update_gear():
        try:
            z1 = int(var_z1.get())
            z2 = int(var_z2.get())
            m  = float(var_module.get())
            bl = float(var_bl.get())
            if z1 < 6 or z2 < 6:
                messagebox.showerror("エラー", "歯数は6以上にしてください")
                return
            if m <= 0:
                messagebox.showerror("エラー", "モジュールは正の値にしてください")
                return
            if bl < 0:
                messagebox.showerror("エラー", "バックラッシュ係数は0以上にしてください")
                return
        except ValueError:
            messagebox.showerror("エラー", "数値を入力してください")
            return

        p1 = calc_gear_params(z1, m, bl)
        p2 = calc_gear_params(z2, m, bl)
        state["p1"] = p1
        state["p2"] = p2

        # 表示ラベル更新
        lbl_da1.config(text=f"⌀{p1['addendum_radius']*2:.2f} mm")
        lbl_da2.config(text=f"⌀{p2['addendum_radius']*2:.2f} mm")
        lbl_cd.config( text=f"{p1['pitch_radius']+p2['pitch_radius']:.2f} mm")

        # 中心距離
        a = p1["pitch_radius"] + p2["pitch_radius"]
        # 両歯車が収まるスケール
        total_w = p1["addendum_radius"] + a + p2["addendum_radius"]
        total_h = max(p1["addendum_radius"], p2["addendum_radius"]) * 2
        state["scale"] = min(CANVAS_W * 0.9 / total_w, CANVAS_H * 0.9 / total_h)
        state["ox"]    = CANVAS_W / 2
        state["oy"]    = CANVAS_H / 2
        redraw()

    def export():
        if state["p1"] is None:
            return
        export_dsm_polyline(state["p1"], "gear1_dsm.txt")
        export_dsm_polyline(state["p2"], "gear2_dsm.txt")
        export_log(state["p1"], state["p2"], "gear1_dsm.log", "gear2_dsm.log")
        messagebox.showinfo("完了",
            f"gear1_dsm.txt / gear2_dsm.txt と\n"
            f"gear1_dsm.log / gear2_dsm.log を出力しました")

    tk.Button(ctrl, text="更新",    command=update_gear,
              font=("Helvetica", 11), padx=8).pack(side="left", padx=(16, 4))
    tk.Button(ctrl, text="DSM出力", command=export,
              font=("Helvetica", 11), padx=8).pack(side="left", padx=4)

    # ── キャンバス ────────────────────────────────
    canvas = tk.Canvas(root, width=CANVAS_W, height=CANVAS_H, bg="black")
    canvas.pack()

    def redraw():
        if state["p1"] is None:
            return
        canvas.delete("all")
        p1, p2  = state["p1"], state["p2"]
        scale   = state["scale"]
        ox, oy  = state["ox"], state["oy"]
        a       = p1["pitch_radius"] + p2["pitch_radius"]

        # 歯車1: 左、歯車2: 右（中心距離 a だけ離す）
        cx1, cy1 = -a / 2, 0.0
        cx2, cy2 =  a / 2, 0.0

        # 噛み合い位相: 歯車2の歯溝中心が -x方向に来るよう設定
        tau2   = p2["tau"]
        phase1 = 0.0
        phase2 = math.pi - tau2 / 2

        draw_gear(canvas, p1, cx1, cy1, phase1, scale, ox, oy)
        draw_gear(canvas, p2, cx2, cy2, phase2, scale, ox, oy)

        # 中心線
        x1s, y1s = ox + cx1 * scale, oy
        x2s, y2s = ox + cx2 * scale, oy
        canvas.create_line(x1s, y1s, x2s, y2s,
                           fill="#444444", width=1, dash=(4, 4))

        draw_legend(canvas)
        draw_scale_bar(canvas, state["scale"], CANVAS_W, CANVAS_H)

    def on_wheel(event):
        factor = ZOOM_FACTOR if event.delta > 0 else 1 / ZOOM_FACTOR
        state["ox"] = event.x + (state["ox"] - event.x) * factor
        state["oy"] = event.y + (state["oy"] - event.y) * factor
        state["scale"] *= factor
        redraw()

    canvas.bind("<MouseWheel>", on_wheel)
    canvas.bind("<Button-4>",   on_wheel)
    canvas.bind("<Button-5>",   on_wheel)

    update_gear()
    root.mainloop()


if __name__ == "__main__":
    main()
