兩個檔案我都完整讀過（`planner.py` 181 行、`mission.py` 836 行）。我用「每一行都解釋」嘅方式寫，但連續嘅初始化會group做一行，否則會太長。

---

# 第一部分：`planner.py`（手腳）

## P.0 一句話

> `planner.py` 收到一串格仔路線，負責「轉方向、直行去下一格中心」。佢唔識迷宮，只識行。

```
set_path(路線)
     │
     ▼
┌───────────────────────────────┐
│ step() 每 10ms 問一次：        │
│   有路？ ──冇→ ARRIVED         │
│   方向錯得多？ → 原地轉         │
│   否則 → 直行去下一格中心       │
└───────────────────────────────┘
```

## P.1 第 1–23 行：檔頭 + import

```python
 1–13: """..."""                       ← 檔案說明（docstring）
15: from __future__ import annotations ← 型別提示延後計算
17: import math                          ← 數學函數（cos, sin, sqrt…）
19: from .config import ControlConfig, RobotConfig
20: from .control import ARRIVED, BLOCKED, DRIVE, STUCK, TURN, Command, CorridorSteering
21: from .kinematics import Pose2D
22: from .maze import cell_centre, wrap_angle
23: from .mouse_map import DIR_TO_HEADING_INDEX, heading_angle
```

| 行 | 意思 | Python 概念 |
|---|---|---|
| 1–13 | 用三個引號寫檔案說明 | `"""docstring"""` |
| 15 | 型別提示唔即時計 | `from __future__` |
| 17 | 借 math 工具 | `import` |
| 19–23 | 借自己 package 其他檔嘅嘢 | 相對匯入 `.xxx` |
| 20 | 借 `Command`（回傳俾 mission 嘅物件）同 `CorridorSteering`（保持置中） | 同一個 package |

## P.2 第 26–33 行：建立 CellPilot

```python
26: class CellPilot:
29:     def __init__(self, control=None, robot=None):
30:         self.cfg = control or ControlConfig()
31:         self.robot = robot or RobotConfig()
32:         self.steering = CorridorSteering(self.cfg)
33:         self.reset()
```

| 行 | 意思 |
|---|---|
| 26 | 定義一個 class（一個「物件藍圖」） |
| 29 | `__init__` = 建立物件時自動執行 |
| 30 | 有傳 `control` 就用，否則用預設 `ControlConfig()`；`A or B` 意思係「A 冇就 B」 |
| 31 | 同上，機器人設定 |
| 32 | 建立「走廊置中」幫手 |
| 33 | 呼叫 `reset()` 初始化所有狀態 |

## P.3 第 35–48 行：`reset()`

```python
36: self.pose = pose or Pose2D()
37: self.path = []
38: self.index = 0
39: self.state = ARRIVED
40: self.leg_heading = self.pose.theta
41: self.turn_target = self.pose.theta
42: self.turn_elapsed = 0.0
43: self.turn_deg = 0.0
44: self._leg_distance = 0.0
45: self.cells_reached = 0
46: self.turn_count = 0
47: self.distance = 0.0
48: self.last_note = "idle"
```

| 行 | 意思 |
|---|---|
| 36 | 記住現在位置（x, y, 角度） |
| 37 | 路線係空 list |
| 38 | 而家去緊第幾個格（0 = 仲未開始） |
| 39 | 狀態 = 「已到達」（即係冇任務） |
| 40–41 | 這段路要向邊（角度）；一開始用而家方向 |
| 42–43 | 轉彎計時器、轉幾多度 |
| 44 | 今段行咗幾遠（安全用） |
| 45–47 | 統計：到咗幾多格、轉咗幾多次、行咗幾多米 |
| 48 | 最後一句提示（俾人睇） |

## P.4 第 50–60 行：`set_path()`

```python
50: def set_path(self, cells, pose=None):
52:     if pose is not None:
53:         self.pose = pose
54:     self.path = list(cells)
55:     self.index = 1 if len(cells) > 1 else 0
56:     self.state = ARRIVED if len(cells) <= 1 else DRIVE
57:     self.turn_elapsed = 0.0
58:     self._leg_distance = 0.0
59:     if len(cells) > 1:
60:         self._begin_leg(cells[0], cells[1])
```

| 行 | 意思 | Python 概念 |
|---|---|---|
| 50 | 設定新路線 | 預設參數 `pose=None` |
| 52–53 | 有傳位置就更新 | `is not None` |
| 54 | 複製一份 list，唔改原本嗰個 | `list(cells)` |
| 55 | 目標索引：兩格以上就由第 1 格開始 | 三元表達式 |
| 56 | 少過兩格 = 已經到；否則開始行 | 三元表達式 |
| 57–58 | 重設轉彎計時、本段距離 |
| 59–60 | 有下一格就計方向 | `_begin_leg` |

**為什麼 `cells[0]` 一定要係「而家所在格」？** 因為第 0 格代表起點，第 1 格才係要行去嘅目標。

## P.5 第 62–70 行：兩個 property

```python
62: @property
63: def arrived(self) -> bool:
64:     return self.state == ARRIVED

66: @property
67: def target_cell(self):
68:     if 0 <= self.index < len(self.path):
69:         return self.path[self.index]
70:     return None
```

| 行 | 意思 |
|---|---|
| 62 | `@property` = 俾人用 `pilot.arrived` 咁讀，唔使加括號 |
| 64 | 狀態係 ARRIVED 就回 True |
| 67–70 | 回傳「下一個目標格」；如果索引超出範圍就回 `None` |

## P.6 第 72–79 行：`_begin_leg()`

```python
72: def _begin_leg(self, cell_from, cell_to):
73:     dc, dr = cell_to[0]-cell_from[0], cell_to[1]-cell_from[1]
74:     for name, (dx, dy) in (("north",(0,1)), ("east",(1,0)),
75:                            ("south",(0,-1)), ("west",(-1,0))):
76:         if (dx, dy) == (dc, dr):
77:             self.leg_heading = heading_angle(DIR_TO_HEADING_INDEX[name])
78:             return
79:     raise ValueError(...)
```

| 行 | 意思 | Python 概念 |
|---|---|---|
| 73 | 計算兩個格嘅差：`dc` = 向東差幾格，`dr` = 向北差幾格 | tuple unpacking、索引 |
| 74–75 | 逐個方向試：北 (0,1)、東 (1,0)、南 (0,-1)、西 (-1,0) | `for ... in` 加 tuple 拆包 |
| 76 | 如果方向吻合 | tuple 比較 |
| 77 | 把方向名轉成角度（北=90°、東=0°、西=180°、南=-90°） | 函數呼叫 |
| 79 | 如果四個方向都唔吻合 → 報錯（兩個格唔相鄰） | `raise ValueError` |

## P.7 第 82–98 行：`step()`（最重要嘅入口）

```python
82: def step(self, pose, readings, dt, speed, turn_rate=None):
85:     if dt <= 0.0:
86:         dt = 1e-3
87:     self.pose = pose
88:     turn_rate = turn_rate or self.cfg.turn_rate
90:     if not self.path or self.state == ARRIVED or self.index >= len(self.path):
91:         self.state = ARRIVED
92:         return Command(0.0, 0.0, ARRIVED, "path complete", finished=True)
94:     err = wrap_angle(self.leg_heading - pose.theta)
95:     coarse = abs(err) > math.radians(self.cfg.drive_heading_limit_deg)
96:     if self.state == TURN or coarse:
97:         return self._turn(pose, err, turn_rate, dt)
98:     return self._drive(pose, readings, dt, speed)
```

| 行 | 意思 | Python 概念 |
|---|---|---|
| 82 | 每個控制週期呼叫一次 | 5 個參數 |
| 85–86 | 如果 dt 係 0 或負數，用 1ms 代替（防止除以 0） | `if`、`1e-3` = 0.001 |
| 87 | 更新自己記住嘅位置 | 屬性賦值 |
| 88 | 冇指定轉速就用預設 | `or` |
| 90 | 冇路 / 已到達 / 索引超界 → 都當完成 | `not`、`or` |
| 91–92 | 設狀態 ARRIVED，回傳「停車」指令 | `return` |
| 94 | 計算「這段路應該面向」同「而家面向」差幾多 | `wrap_angle` 把角度限制在 ±180° |
| 95 | 誤差是否大過 30°（`drive_heading_limit_deg`） | `abs()`、`radians()` |
| 96–97 | 如果正在轉彎，或者方向錯得遠 → 轉彎 | `or` |
| 98 | 否則 → 直行 | `return` |

### `Command(...)` 係咩？

```python
Command(v, w, state, note, info={}, finished=False)
```

- `v` = 前進速度
- `w` = 轉向速度
- `state` = 而家做緊咩（DRIVE/TURN/ARRIVED…）
- `note` = 給人睇嘅說明
- `info` = 額外資料（例如呢段去邊）
- `finished` = 是否完成

## P.8 第 100–120 行：`_turn()`（原地轉）

```python
100: def _turn(self, pose, err, turn_rate, dt):
101:     if self.state != TURN:
102:         self.state = TURN
103:         self.turn_elapsed = 0.0
104:         self.turn_target = self.leg_heading
105:         self.turn_deg = wrap_angle(self.leg_heading - pose.theta)
106:         self.turn_count += 1
108:     self.turn_elapsed += dt
109:     err = wrap_angle(self.turn_target - pose.theta)
110:     tol = math.radians(self.cfg.turn_tol_deg)
111:     if abs(err) <= tol or self.turn_elapsed > self.cfg.turn_timeout:
112:         self.state = DRIVE
113:         return Command(0.0, 0.0, DRIVE, f"turn done, ...")
115:     mag = abs(err) * self.cfg.kp_turn
116:     if abs(err) > math.radians(8.0):
117:         mag = max(mag, turn_rate * 0.75)
118:     mag = min(mag, turn_rate)
119:     return Command(0.0, math.copysign(mag, err), TURN, ...)
```

| 行 | 意思 |
|---|---|
| 101 | 如果未開始轉（第一次入嚟） |
| 102–106 | 設狀態 TURN、記目標角度、記原本差幾度、轉彎次數 +1 |
| 108 | 累積轉彎時間 |
| 109 | 重新計算仲差幾多度 |
| 110 | 容差 1.2° |
| 111 | 差少過 1.2° **或者** 轉超過 2.5 秒 → 結束 |
| 112–113 | 轉完，回傳「停車」指令（下一步會開始直行） |
| 115 | 轉速與誤差成正比（誤差大 → 轉快） |
| 116–117 | 如果差過 8°，最少用 75% 最高轉速（避免太慢） |
| 118 | 但唔可以超過最高轉速 |
| 119 | `copysign` = 跟住 err 嘅正負號，決定向左定右轉 |

## P.9 第 122–177 行：`_drive()`（直行）

```python
122: def _drive(self, pose, readings, dt, speed):
123:     target = self.path[self.index]
124:     tx, ty = cell_centre(*target)
125:     ux, uy = math.cos(self.leg_heading), math.sin(self.leg_heading)
126:     along = (tx-pose.x)*ux + (ty-pose.y)*uy
127:     lateral = -(tx-pose.x)*uy + (ty-pose.y)*ux
```

| 行 | 意思 |
|---|---|
| 123 | 下一個目標格 |
| 124 | 目標格中心嘅 (x, y)；`*target` 把 `(c,r)` 拆成兩個參數 |
| 125 | 車頭方向嘅單位向量（cos, sin） |
| 126 | **along** = 沿住車頭方向，仲有幾多米到目標 |
| 127 | **lateral** = 偏離中線幾多米（左右） |

**點解要分 along / lateral？**
- `along` 用嚟判斷「到咗未」。
- `lateral` 用嚟知道自己有冇偏離走廊。

### 安全檢查 1：前面有地圖唔知嘅牆

```python
134: t = readings.front
135: if (self.state == DRIVE and math.isfinite(t) and along > 0.04
136:         and t < along - 0.060):
137:     self.state = BLOCKED
138:     return Command(0.0, 0.0, BLOCKED, ..., finished=False)
```

| 行 | 意思 |
|---|---|
| 134 | 前置感測器讀到嘅距離（米） |
| 135 | 條件：正在直行、讀數有效（唔係 inf）、距離目標中心超過 4cm |
| 136 | 而且牆嘅距離 **細過** `along - 60mm` |
| 137 | 代表牆喺下一格中心**之前** → 地圖錯 → 狀態 BLOCKED |
| 138–142 | 回傳「停車」+ 額外資料（呢段路、讀數、along），等 mission 處理 |

**為什麼要 `- 0.060`？**
感測器裝喺車中心前面 48mm；正常下一格中心前面嘅牆會讀到約 `along + 36mm`。如果讀到 `along - 60mm`，即係幅牆真係喺路中間。

### 安全檢查 2：到咗未？

```python
144: if along <= 0.0:
145:     self.cells_reached += 1
146:     self.index += 1
147:     self._leg_distance = 0.0
148:     if self.index >= len(self.path):
149:         self.state = ARRIVED
150:         return Command(0.0, 0.0, ARRIVED, "path complete", finished=True)
151:     self._begin_leg(self.path[self.index-1], self.path[self.index])
152:     return self._turn(pose, wrap_angle(self.leg_heading - pose.theta), self.cfg.turn_rate, dt)
```

| 行 | 意思 |
|---|---|
| 144 | `along <= 0` = 已經行過目標中心 |
| 145–146 | 計一格、去下一個目標 |
| 147 | 重設本段距離 |
| 148–150 | 如果冇下一格 → 完成 |
| 151 | 有下一格 → 計算新方向 |
| 152–153 | 即刻進入轉彎判斷（可能唔需要轉，會即刻完成） |

### 第 155–157 行：速度

```python
155: v = min(speed, math.sqrt(max(2.0 * self.cfg.decel * along, 0.0)))
156: if along > 0.04:
157:     v = max(v, self.cfg.slow_speed)
```

- `sqrt(2 × 減速度 × 距離)` = 煞車公式：距離越短，最高速度越低。
- `min(speed, ...)`：唔可以超過要求速度。
- 如果仲有 4cm 以上，最少用 `slow_speed`（0.12 m/s）慢慢行。

### 第 159–161 行：轉向 + 計距離

```python
159: w, source, wall_err = self.steering.compute(readings, pose.theta, self.leg_heading)
160: self.distance += abs(v) * dt
161: self._leg_distance += abs(v) * dt
```

- `steering.compute` 回傳「要轉幾多」+ 用咗左／右牆 + 牆距離誤差。
- 更新總距離、本段距離。

### 第 166–170 行：行得太遠 = 出事

```python
166: if self._leg_distance > self.cfg.leg_overrun:
167:     self.state = STUCK
168:     return Command(0.0, 0.0, STUCK, "leg overrun ...", finished=True)
```

正常一段 180mm；行到 320mm 都未到中心 → 一定有問題 → 停。

### 第 171–177 行：回傳指令

```python
171: self.last_note = f"drive {起點}->{目標} along {...} lateral {...} wall {...}"
174: return Command(v, w, DRIVE, self.last_note, {"leg": ..., "along": ..., ...})
```

- 記錄一句人類可讀嘅說明。
- 回傳 `Command`，包含 v、w、狀態、說明、額外資料。

## P.10 第 179–181 行：`summary()`

```python
179: def summary(self):
180:     return f"{self.state}: {self.cells_reached} cells, {self.turn_count} turns, {self.distance:.2f} m driven"
```

一句總結，方便印 log。

---

# 第二部分：`mission.py`（大腦）

## M.0 一句話

> `mission.py` 係領隊：決定幾時搜尋、幾時返屋企、幾時做決賽，並且保存地圖同報告。

```
每 10ms：
 ┌──────────────────────────────────────┐
 │ 1. 我到咗格中心嗎？ → 寫牆上地圖        │
 │ 2. 地圖/任務有變嗎？ → _plan() 重新計劃 │
 │ 3. 叫 planner 行                      │
 │ 4. planner 回報：BLOCKED / STUCK / 到咗 │
 └──────────────────────────────────────┘
```

## M.1 第 1–44 行：檔頭、import、phase 常數

```python
22: from __future__ import annotations
24: import math
25: import time
26: from dataclasses import dataclass, field
28–36: 從自己 package 匯入設定、Command、floodfill、位置、迷宮、地圖、pilot
39: SEARCH = "search"
40: HOME = "home"
41: SPEED_OUT = "speed_out"
42: SPEED_HOME = "speed_home"
43: FINAL = "final"
44: DONE = "done"
```

| 行 | 意思 |
|---|---|
| 24–26 | 數學、時間、dataclass 工具 |
| 28–36 | 相對匯入：設定、Command 類、flood 函數、Pose2D、迷宮常數、MouseMap、CellPilot |
| 39–44 | **階段名稱**。用字串常數而唔用數字，因為除錯時易睇 |

**為什麼唔直接用 `"search"`？** 打錯字好難查；用 `SEARCH` 一改就全改。

## M.2 第 46–57 行：Trail 顏色

```python
48: TRAIL_COLOURS = {
49:     ("run", 1): (0.05, 0.30, 1.00),   # 第 1 次搜尋：藍
50:     ("run", 2): (0.72, 0.53, 0.04),   # 第 2 次：深黃
51:     ("run", 3): (0.00, 0.65, 0.15),   # 第 3 次：綠
52:     ("run", 4): (0.55, 0.15, 0.75),   # 之後：紫
53:     ("attempt", 1): (1.00, 0.55, 0.00),  # 計時賽：橙
54:     ("final", 0): (0.95, 0.05, 0.05),    # 決賽：紅
55:     ("other", 0): (0.55, 0.55, 0.55),    # 其他：灰
56: }
57: TRAIL_DEFAULT_COLOUR = (0.55, 0.55, 0.55)
```

- dict（字典）：key = `(類型, 第幾次)`，value = RGB 顏色。
- RViz 用嚟畫唔同顏色嘅路線。

## M.3 第 60–79 行：`trail_key()` / `trail_colour()`

```python
67: if phase in (SEARCH, HOME): return ("run", int(attempt))
69: if phase in (SPEED_OUT, SPEED_HOME): return ("attempt", int(speed_index))
71: if phase == FINAL: return ("final", 0)
73: return ("other", 0)

78: return TRAIL_COLOURS.get(trail_key(...), TRAIL_DEFAULT_COLOUR)
```

| 行 | 意思 |
|---|---|
| 67–68 | 搜尋同返屋企用同一個 key → 同一條 trail |
| 69–70 | 計時賽用 speed_index |
| 71–72 | 決賽專用 |
| 78 | `dict.get(key, 預設值)`：搵唔到就用灰色 |

## M.4 第 82–94 行：`RunRecord`

```python
82: @dataclass
83: class RunRecord:
84:     kind: str
85:     index: int
86:     cells: int
87:     distance: float
88:     duration: float
89:     completeness: float
90:     note: str = ""

92:     def line(self):
93:         return (f"{self.kind:<8} {self.index:<3} ...")
```

| 行 | 意思 | Python 概念 |
|---|---|---|
| 82 | `@dataclass` 自動幫你寫 `__init__` | decorator |
| 84–90 | 每一項資料（欄位）同型別 | 類別屬性 |
| 90 | 預設值空字串 | 預設參數 |
| 92–94 | 變成一條表格行 | f-string、`:<8` 左對齊、`:>8.2f` 右對齊兩位小數 |

## M.5 第 97–108 行：`MissionRunner.__init__`

```python
100: def __init__(self, mission=None, control=None, robot=None):
103:     self.cfg = mission or MissionConfig()
104:     self.control = control or ControlConfig()
105:     self.robot = robot or RobotConfig()
106:     self.pilot = CellPilot(self.control, self.robot)
107:     self.mmap = MouseMap()
108:     self.reset()
```

- `self.mmap = MouseMap()`：**老鼠自己嘅地圖**（一開始全部 UNKNOWN）。
- `self.pilot`：手腳。

## M.6 第 111–158 行：`reset()`（全部狀態歸零）

| 行 | 意思 |
|---|---|
| 112–114 | 位置、pilot 重置 |
| 115–117 | 時間 0、階段 SEARCH、第 1 次 |
| 118 | 準備本次搜尋（探索隊列） |
| 119 | 計時賽索引 |
| 120–122 | 每次 run 嘅記錄、事件、log |
| 123–126 | 距離表、目標、目標類型、已計劃版本 |
| 127–128 | 路線 |
| 129–131 | 上一次寫地圖嘅格、本次 run 格數 |
| 132 | 未處理嘅地圖改動（給 incremental flood） |
| 133–138 | 終點距離場 + 更新計數 |
| 139 | 去過嘅格 |
| 141–147 | 探索隊列、預算、去程格、要避開格、run log |
| 148–151 | 死鎖偵測、卡住恢復次數、blocked 次數 |
| 152–155 | 本次 run 距離、開始時間 |
| 156–158 | 目標到過未、完成未、提示 |

**重點概念：**

```python
self._outbound_cells = set()
```

`set` = 集合，唔准有重複，查「有冇喺入面」好快。

## M.7 第 163–179 行：`_at_centre()`（係唔係格中心）

```python
173: cell = to_cell(pose.x, pose.y)
174: cx, cy = cell_centre(*cell)
175: dx, dy = pose.x - cx, pose.y - cy
176: ux, uy = math.cos(pose.theta), math.sin(pose.theta)
177: along = dx*ux + dy*uy
178: lateral = -dx*uy + dy*ux
179: return abs(along) <= map_along_tol and abs(lateral) <= map_lateral_tol
```

| 行 | 意思 |
|---|---|
| 173 | 我而家喺邊一格 |
| 174 | 該格中心座標 |
| 175 | 我同中心嘅差 |
| 176 | 車頭方向單位向量 |
| 177 | 沿車頭方向差幾多米（前後） |
| 178 | 垂直方向差幾多米（左右） |
| 179 | 前後 ±20mm、左右 ±50mm 之內 → 當「喺中心」 |

**為什麼係長方形？** 前感測器喺前面 48mm；太早／太遲讀會令同一幅牆由 39mm 變 84mm，容易誤判。左右偏移影響細，所以可以鬆。

## M.8 第 181–183 行：`_aligned()`

```python
182: deg = math.degrees(pose.theta) % 90.0
183: return min(deg, 90.0 - deg) < tol_deg
```

- 把角度除以 90° 取餘數 → 得到「距離最近嘅正東南西北幾多度」。
- 少過 12° 就當「對正」。

例：93° → 93 % 90 = 3°；3 < 12 → 對正。

## M.9 第 185–192 行：`_say()` / `pop_events()`

```python
186: self.events.append(text)
187: self.log.append((self.elapsed, text))
190: out = self.events
191: self.events = []
192: return out
```

- `_say` = 記錄一句事件（同時記時間）。
- `pop_events` = 拿走所有未讀事件，然後清空（給 ROS node 印 log）。

## M.10 第 197–213 行：`_explore_step()`（舊式全圖探索）

```python
205: targets = [所有未去過嘅格（除咗自己）]
207: if not targets: return None
209: dist = flood_fill(mmap, targets)
210: if 自己去唔到: return None
212: _, nxt = best_step(mmap, dist, cell, heading_idx)
213: return nxt
```

呢個係 `strategy="full_explore"` 用嘅舊方法：由**所有未去過嘅格**一齊 flood，然後行去最近嘅未探索區。

## M.11 第 215–250 行：`_choose_target()`（決定去邊）

```python
226: if self.phase == SEARCH:
227:     if strategy == "full_explore": ...        ← 舊式全圖探索
234:     cell = ...
237:     if 有探索目標 且 未用完 budget:
238–239:     return 探索目標
240–241:     if 終點可達: return 終點
242–246:     if 終點不可達 且 允許: return frontier
247:     return 終點
248–249: if 階段係 HOME/SPEED_HOME: return 起點
250: return 終點
```

**決策次序（好重要）：**

1. 有未探索目標、budget 未用完 → 去探索
2. 終點可達 → 去終點
3. 終點不可達 → 去「已知世界邊界」（frontier）
4. 都唔得 → 照去終點（讓 pilot 試）

## M.12 第 252–290 行：`_frontier_step()`

```python
266: n = self.mmap.n
267: to_frontier = flood_fill(self.mmap, [cell])   ← 由我行去每個格要幾步
268: best, best_score = None, None
269–270: 逐格
271:     如果未去過 或 冇未知邊 → 跳過
273:     here = 行去嗰度要幾步
274–275:     去唔到 → 跳過
276:     there = 由嗰度去終點要幾步
277–282:     如果去唔到 → 用曼哈頓距離估計
283:     score = here + there
284–285:     如果更低分 → 記住
286–287: 冇候選 → None
288–289: 由最優先 frontier 再 flood，行第一步
```

**曼哈頓距離** = `|x1-x2| + |y1-y2|`，即係「唔准斜行」嘅格數。

## M.13 第 292–301 行：`_has_unknown_edge()`

```python
295: for name in ("north","east","south","west"):
296:     dc, dr = DELTAS[name]
297:     nc, nr = c+dc, r+dr
298:     if 喺地圖範圍內:
299–300:     if 呢條邊係 UNKNOWN → True
301: return False
```

即係：呢格仲有冇未睇過嘅邊？

## M.14 第 303–395 行：`_plan()`（核心）

```python
312: cell = to_cell(self.pose.x, self.pose.y)
313: goal_targets = tuple(GOAL_CELLS)
```

### 第 316–325 行：維持終點距離場

```python
316: if 未有終點場 或 目標改咗:
317–320:     全圖 flood_fill，記低版本
321: elif 有未處理改動:
322–325:     update_distances（只補受影響格）
```

### 第 330–336 行：防止「舊數值講大話」

```python
330: if 地圖版本 != 場版本 而且 場話去唔到:
332:     強制全圖重算
```

**為什麼？** 因為 incremental 更新可能未完成；如果憑舊場話「冇路」，就會錯誤停機。所以「冇路」時要再全算一次確認。

### 第 338–341 行：問「去邊」

```python
338: goal_reachable = goal_field[cell] < 255
339: targets, kind, why = self._choose_target(goal_reachable)
```

### 第 343–364 行：揀用邊個距離場

```python
344–345: kind == "goal"  → 用 goal_field
346–360: kind == "home" 而且 有 avoid_cells
             → flood_fill_weighted（避開去程，只行已知路）
             → 如果冇路，fallback 普通 flood
361–363: 其他 → 由目標格 flood
364: self.distances = field
```

### 第 366–382 行：如果連自己格都去唔到

```python
366: if field[cell] >= UNREACHABLE:
367–377:     如果係探索目標 → 刪除嗰個目標，再 plan 一次（遞迴）
378–382:     否則 → 停低，回傳 False
```

**遞迴**：`return self._plan(force)` = 「再叫自己做一次」。因為刪除咗一個目標，情況已經唔同，可以再試。

### 第 383–395 行：拉出路線交給 pilot

```python
383: path = path_to(self.mmap, field, cell)
384–388: 如果只有一格 → 已經到，回 False
389–391: 如果係搜尋去終點 → 記低去程格（返程要避開）
392: pilot.set_path(path, self.pose)
393: 記低已計劃版本
394: last_note
395: return True
```

## M.15 第 400–526 行：`step()`（每 10ms 主循環）

```python
410: self.pose = pose
411: map_pose = map_pose or pose
412: self.elapsed += dt
414–415: 如果完成 → 回 DONE
```

### 第 417–434 行：學牆

```python
418: cell = to_cell(pose.x, pose.y)          ← 而家位置
419: map_cell = to_cell(map_pose.x, map_pose.y) ← 影相時位置
420: mapping_ok = (上次唔係轉彎) and (對正方向)
421: if mapping_ok and 喺格中心 and 呢格未寫過:
422:     hd = heading_index(map_pose.theta)
423–428: update_from_sensors(前/左/右牆)
430:     self._pending_changes.extend(changed)   ← 排隊俾 incremental flood
431–434: 記住寫過呢格、本次 run 格數、去過格 list
```

### 第 436–439 行：需要重新計劃嗎？

```python
if pilot 已完成 或 地圖版本變咗 或 第一次:
    self._plan()
```

**地圖每格都變 → 所以幾乎每格都重新計劃。** 呢個就係「每格中心都係決策點」嘅原因。

### 第 441–448 行：叫 pilot 行

```python
442: speed = self._speed_for_phase()
443–444: turn_rate = 決賽/計時賽用 3.5，其餘用 3.0
445: cmd = self.pilot.step(pose, readings, dt, speed, turn_rate)
446–448: 記狀態、格數、距離
```

### 第 450–469 行：死鎖保護

```python
453–457: 如果移動超過 5mm → 更新「最後移動時間」
458:     如果超過 3 秒冇移動:
459:         重試次數 +1
463:         _remap_here(readings)  ← 用現在感應器重新寫呢格
464:         _plan()
465:         重設計時
466–469:     超過 3 次 → STUCK，結束
```

### 第 471–474 行：pilot 報 STUCK

```python
471–474: 印出來、結束
```

### 第 476–522 行：pilot 報 BLOCKED（地圖錯）

```python
486: leg = cmd.info.get("leg")   ← 呢段由邊格去邊格
488–492: 計方向名（東/西/南/北）
493–494: 記住地圖原本話咩
495–498: 如果 legacy_block_handling（舊 bug 重現用）→ 唔改地圖
499–506: 否則：
500:     mmap.set(..., WALL)        ← 寫入真牆
505:     _pending_changes.extend(...) ← 排隊更新 flood
506:     _say(_blocked_note(...))
507–508: 如果方向唔明 → 記錄
509: blocked_events += 1
510–522: 重新 plan；如果都冇路 → remap 再 plan；再冇 → 結束
```

**重點：即時感應器贏過舊地圖。** 唔改地圖就會永遠撞同一幅牆。

### 第 524–526 行：pilot 報 ARRIVED

```python
524–525: self._on_arrival(cell)
526: return cmd
```

## M.16 第 529–545 行：`_remap_here()`

```python
535–536: 唔係格中心 → 唔做（感應器讀數唔可信）
537: cell = 而家格
538–542: 用前/左/右感應器覆寫呢格三條邊
544: 排隊俾 incremental flood
545: 記住已寫
```

## M.17 第 547–566 行：`_blocked_note()`

```python
555–556: t = 感應器距離, along = 距離下一格中心
557–558: 基本說明
559–565: 如果地圖原本話「通」(was=False)：
             implied = t - 0.036
             如果 > 25mm → 加一句「呢個讀數比格邊遠，可能係位置/地圖錯，唔係牆」
```

呢個係「診斷訊息」：幫你分辨「真牆」同「位置估錯」。

## M.18 第 569–576 行：`_speed_for_phase()`

| 階段 | 速度 |
|---|---|
| SEARCH | `search_speed`（0.30） |
| HOME | `search_speed × 1.2`（0.36） |
| FINAL | `final_speed`（0.30） |
| 其他（計時賽） | `speed_run_speed`（0.40） |

## M.19 第 579–640 行：`_on_arrival()`（狀態機）

```python
584: if 階段 == SEARCH:
585–589:     如果到終點 → 記錄、_begin_return()、轉 HOME
590–601:     如果到探索目標 → 扣 budget、pop、補充新目標、再 plan
602–604:     其他（frontier）→ 再 plan
607–621: if 階段 == HOME:
608:         記錄
609–612:     未夠 search_runs → 下一次 SEARCH
613–620:     夠 → 有 speed_runs 就 SPEED_OUT；否則 FINAL
623–626: if 階段 == SPEED_OUT → 轉 SPEED_HOME
628–636: if 階段 == SPEED_HOME → 記錄；未夠就再 SPEED_OUT；夠就 FINAL
638–640: if 階段 == FINAL → 記錄、finish_mission()
```

**用表睇：**

| 階段 | 到達 | 下一步 |
|---|---|---|
| SEARCH | 終點 | HOME |
| SEARCH | 探索格 | 再 SEARCH（扣 budget） |
| HOME | 起點 | 下一個 SEARCH 或 FINAL |
| FINAL | 終點 | 完成 |

## M.20 第 642–651 行：`finish_mission()`

```python
643–644: 已完成 → 唔做
645–648: 計最優路線（兩個分支其實一樣，係一個小重複）
649–651: 設 finished、phase=DONE、記錄
```

> 誠實講：645–648 兩個分支內容完全一樣，係一段多餘嘅 `if/else`，唔影響行為。

## M.21 第 654–673 行：`_begin_search_run()`

```python
663–666: 清空探索隊列、budget、去程格
667–668: 如果 budget <= 0 或 第 1 次搜尋 → 唔探索，直接返回
669: targets = _unexplored_targets()
670–673: 有目標就排隊 + 印訊息
```

## M.22 第 675–687 行：`_unexplored_targets()`

```python
681: cell = 而家格
682: dist = flood_fill(mmap, [cell])        ← 由我行去每格幾步
683: n = 16
684–685: 揀「未去過」而且「去到」嘅格
686: 按距離由近到遠排序
687: 取最近 N 個（預設 6）
```

## M.23 第 689–697 行：`_begin_return()`

```python
691: if alternate_return 而且 唔係最後一次搜尋:
692:     _avoid_cells = copy(去程格)
694–695:     印訊息
696–697: else: 清空（唔罰任何格）
```

## M.24 第 699–707 行：`_start_phase()`

```python
700: phase = 新階段
701: 記開始時間
702–704: 重設 pilot 格數、距離、本次 run 格數
705: 記錄事件
706: 立即重新計劃
707: 記錄「呢個階段嘅第一條路」
```

## M.25 第 709–721 行：`_record_run()`

```python
710: duration = 而家 - 階段開始
711–718: 建立 RunRecord（kind、index、格數、距離、時間、地圖完成度、備註）
719–721: 記錄一句總結
```

`cells=max(_cells_this_run, pilot.cells_reached)`：取兩者較大值，因為兩個計數方法可能有少少差異。

## M.26 第 726–836 行：報告

### `optimal_route()`（726–730）

```python
728: 由終點 flood
729: 由起點拉最短路線
730: 回傳 (路線, 距離表)
```

### `table_header()` / `runs_table()`（732–737）

- 標題行。
- `"\n".join(...)` 把多行串成一個字串。

### `route_grid()`（739–754）

```python
744: 由 r=15 到 0（由上而下畫）
747–748: 終點畫 " G "
749–750: 起點畫 " S "
752:     路線格畫 " * "，其他畫 " . "
```

### `report()`（756–815）

| 行 | 內容 |
|---|---|
| 758–759 | 計最優路線、格數 |
| 761–763 | 標題 |
| 764–777 | maze seed、mission 設定、策略、搜尋格數、flood 統計 |
| 778–781 | 有 blocked / stuck 才印 |
| 782–787 | 感測器、地圖完成度、最優路線長度 |
| 788–799 | 如果有 truth（驗證用）→ 比較真假地圖、最短路線 |
| 801–802 | attempts 表 |
| 804–805 | 老鼠記憶中嘅迷宮（ASCII） |
| 807–808 | 由終點計嘅距離表 |
| 810–811 | 最優路線圖 |
| 813 | 路線格列表 |

### `write_report()`（817–824）

```python
819: try:
820–821:     with open(path, "w") as fh: fh.write(text)
822–823: except OSError: 回傳錯誤訊息
```

- `with open(...)` 會自動關檔。
- `try/except` 防止寫檔失敗時整個程式崩潰。

### `_truth_as_mouse_map()`（827–836）

```python
829: 建立 MouseMap（16×16）
830–832: 把真實迷宮嘅水平牆抄入去
833–835: 把垂直牆抄入去
836: 回傳
```

呢個只係**驗證用**：把「真實迷宮」變成「全知地圖」，用來對比老鼠嘅記憶。老鼠自己永遠唔會用呢個。

---

# 三個檔案嘅 Python 概念總表

| 概念 | 出現 | 白話 |
|---|---|---|
| `class` / `self` | 兩個檔案 | 物件藍圖；`self` = 呢個物件自己 |
| `__init__` | P29 / M100 | 建立物件時自動執行 |
| `@dataclass` | M82 | 自動寫 `__init__` 嘅 class |
| `@property` | P62 / P66 | 當屬性咁讀，唔使括號 |
| 預設參數 `= None` | 多處 | 冇傳就用預設 |
| `A or B` | 多處 | A 係「假」就用 B |
| 三元 `A if C else B` | 多處 | C 真就 A，否則 B |
| `list[]` / `dict{}` / `set()` / `tuple()` | 多處 | 四種容器 |
| `for ... in ...` | 多處 | 逐個處理 |
| `while` | 多處 | 條件成立就一直做 |
| `if / elif / else` | 多處 | 條件分支 |
| `continue` / `break` | 多處 | 跳過今次 / 跳出迴圈 |
| `return` | 多處 | 回傳結果並結束函數 |
| f-string `f"..."` | 多處 | 字串內插入變數 |
| 格式 `:<8` `:>5.2f` | M93 | 對齊、小數位 |
| `max()` / `min()` / `abs()` | 多處 | 最大、最小、絕對值 |
| `math.hypot` | M455 | 兩點距離 |
| `math.degrees` / `radians` | 多處 | 弧度↔角度 |
| `enumerate` | P125 | 同時拿索引和值 |
| `zip` | M280 | 兩個序列配對 |
| `lambda` | M282 / M686 | 即場細函數 |
| `sorted` / `.sort(key=...)` | M686 | 按指定規則排序 |
| `dict.get(k, default)` | M492 / M752 | 搵唔到就回預設 |
| `getattr(obj, name, default)` | M764 | 安全讀屬性 |
| `with open(...)` | M820 | 自動關檔 |
| `try/except` | M819 | 捕捉錯誤 |
| `set` / `.add` / `.discard` | floodfill | 集合操作 |
| `deque` / `.popleft()` | floodfill | 快速排隊 |
| `heapq` | floodfill | 優先佇列 |
| `continue` | 多處 | 跳過 |
| 遞迴 `self._plan()` | M377 | 自己叫自己 |

---

# 建議嘅學習次序

1. **先睇 `planner.py` P.7–P.9**：`step → _turn → _drive`，因為最直觀。
2. **再睇 `mission.py` M.15 `step()`**：知道每 10ms 發生咩事。
3. **然後 M.14 `_plan()`**：知道路線點嚟。
4. **最後 M.19 `_on_arrival()`**：知道整個比賽流程。
5. 用 `tools/headless_check.py --trace` 邊跑邊睇 log，將 code 同實際行為對照。

---

# 一頁總結

```
planner.py：
  收到路線 → step() → 要轉就 _turn()，唔使轉就 _drive()
  _drive()：計 along/lateral → 檢查牆 → 到咗就下一格 → 否則行 + 置中

mission.py：
  每 10ms：
    喺格中心？ → 寫牆
    地圖變？ → _plan()（flood → 選目標 → path_to → 交給 pilot）
    pilot 行
    BLOCKED → 改地圖再 plan
    ARRIVED → 下一個階段
  階段：SEARCH → HOME → SEARCH → ... → FINAL → 報告
```
