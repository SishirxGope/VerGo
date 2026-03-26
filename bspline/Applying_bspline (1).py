"""
=============================================================
  Autonomous Car Simulation  v3
  ─────────────────────────────────────────────────────────
  Changes from v2:
  1. NPCs enter naturally from the RIGHT edge of the window
  2. Ego stays in the new lane after overtaking (no forced return)
  3. Overtake direction is randomized (left OR right), collision-safe

  Requirements:  pip install pygame numpy
  Controls:  S = spline preview | R = reset | ESC = quit
=============================================================
"""

import sys, math, random
import numpy as np
import pygame

# ── Screen ─────────────────────────────────────────────────
W, H        = 1200, 680
FPS         = 60
NUM_LANES   = 4
LANE_H      = 120
ROAD_TOP    = (H - NUM_LANES * LANE_H) // 2 + 10
ROAD_BOT    = ROAD_TOP + NUM_LANES * LANE_H

# ── Colours ────────────────────────────────────────────────
C_SKY      = (15,  20,  30)
C_ROAD     = (42,  44,  48)
C_ROAD_ALT = (38,  40,  44)
C_EDGE     = (255, 200,   0)
C_DASH     = (210, 210, 210)
C_SHOULDER = (70,  65,  55)
C_WHITE    = (255, 255, 255)

PALETTES = {
    "ego":  ((30,  110, 255), (10,   60, 180), (120, 200, 255)),
    "red":  ((215,  45,  45), (130,  10,  10), (255, 140, 140)),
    "lime": ((40,  190,  80), (15,  110,  35), (140, 255, 160)),
    "gold": ((230, 170,   0), (140,  95,   0), (255, 230, 100)),
}

# ── Physics / AI ───────────────────────────────────────────
EGO_SPEED        = 5.0
NPC_SPEED_RANGE  = (2.2, 3.8)
SAFE_GAP         = 170
WARN_GAP         = 250
OVERTAKE_GAP     = 220
MERGE_SAFE_FRONT = 210
MERGE_SAFE_REAR  = 170

# ──────────────────────────────────────────────────────────
#  HELPERS
# ──────────────────────────────────────────────────────────
def lane_y(idx):
    return ROAD_TOP + idx * LANE_H + LANE_H // 2

def catmull_rom(p0, p1, p2, p3, n=60):
    p0, p1, p2, p3 = map(np.asarray, [p0, p1, p2, p3])
    pts = []
    for i in range(n):
        t = i/(n-1); t2 = t*t; t3 = t2*t
        pt = 0.5*(2*p1+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3)
        pts.append(pt.tolist())
    return pts

def build_spline(wps, n=55):
    wp = [wps[0]] + list(wps) + [wps[-1]]
    path = []
    for i in range(1, len(wp)-2):
        seg = catmull_rom(wp[i-1], wp[i], wp[i+1], wp[i+2], n)
        if i > 1: seg = seg[1:]
        path.extend(seg)
    return path

# ──────────────────────────────────────────────────────────
#  CAR
# ──────────────────────────────────────────────────────────
class Car:
    L  = 78    # length along X (travel direction)
    CW = 38    # width  along Y (across lane)

    def __init__(self, x, y, lane, palette_key, speed=EGO_SPEED, is_ego=False):
        self.x           = float(x)
        self.y           = float(y)
        self.lane        = lane
        self.pal         = PALETTES[palette_key]
        self.speed       = speed
        self.is_ego      = is_ego
        self.state       = "cruise"
        self.target_lane = lane
        self.path        = []
        self.path_idx    = 0
        self.on_spline   = False
        self.blinker     = 0
        self.blink_t     = 0

    @property
    def rect(self):
        return pygame.Rect(self.x - self.L//2, self.y - self.CW//2, self.L, self.CW)

    def draw(self, surf, cam):
        sx = int(self.x - cam)
        sy = int(self.y)
        L, CW = self.L, self.CW
        body, dark, accent = self.pal

        # Shadow
        shad = pygame.Surface((L+6, CW+6), pygame.SRCALPHA)
        pygame.draw.rect(shad, (0,0,0,55), shad.get_rect().inflate(-2,-2), border_radius=6)
        surf.blit(shad, (sx-L//2-1, sy-CW//2+6))

        # Body
        br = pygame.Rect(sx-L//2, sy-CW//2, L, CW)
        pygame.draw.rect(surf, body, br, border_radius=8)
        pygame.draw.rect(surf, dark, br, 2, border_radius=8)

        # Hood (front-right)
        pygame.draw.rect(surf, dark, (sx+L//2-26, sy-CW//2+4, 22, CW-8), border_radius=5)

        # Windscreen
        ws = pygame.Rect(sx+L//2-50, sy-CW//2+5, 26, CW-10)
        wss = pygame.Surface((ws.w, ws.h), pygame.SRCALPHA)
        wss.fill((140, 200, 255, 160))
        surf.blit(wss, (ws.x, ws.y))
        pygame.draw.rect(surf, dark, ws, 1, border_radius=3)

        # Roof
        rf = pygame.Rect(sx-L//2+18, sy-CW//2+6, L-44, CW-12)
        rfs = pygame.Surface((rf.w, rf.h), pygame.SRCALPHA)
        rfs.fill((*dark, 145))
        surf.blit(rfs, (rf.x, rf.y))

        # Rear window
        rw = pygame.Rect(sx-L//2+4, sy-CW//2+5, 18, CW-10)
        rws = pygame.Surface((rw.w, rw.h), pygame.SRCALPHA)
        rws.fill((100, 160, 220, 120))
        surf.blit(rws, (rw.x, rw.y))

        # Wheels
        wl, ww = 18, 10
        for wx, wy in [
            (sx-L//2+6,    sy-CW//2-ww+2),
            (sx+L//2-6-wl, sy-CW//2-ww+2),
            (sx-L//2+6,    sy+CW//2-2),
            (sx+L//2-6-wl, sy+CW//2-2),
        ]:
            pygame.draw.rect(surf, (18,18,18), (wx, wy, wl, ww), border_radius=3)
            pygame.draw.rect(surf, (80,80,90), (wx+4, wy+2, wl-8, ww-4), border_radius=2)

        # Headlights (front-right)
        hl_x   = sx + L//2 - 6
        hl_col = (255,255,180) if self.is_ego else (255,190,110)
        for hy in (sy-CW//2+5, sy+CW//2-5):
            pygame.draw.ellipse(surf, hl_col, (hl_x-5, hy-3, 10, 7))

        # Tail lights (rear-left)
        tl_x = sx - L//2 + 4
        for hy in (sy-CW//2+5, sy+CW//2-5):
            pygame.draw.ellipse(surf, (200,40,40), (tl_x-3, hy-3, 10, 7))

        # Blinker
        self.blink_t = (self.blink_t + 1) % 40
        if self.blinker != 0 and self.blink_t < 20:
            by = (sy-CW//2-5) if self.blinker == -1 else (sy+CW//2-3)
            pygame.draw.rect(surf, (255,160,0), (sx+L//2-30, by, 24, 6), border_radius=2)

        # Ego glow
        if self.is_ego:
            glow = pygame.Surface((L+22, CW+22), pygame.SRCALPHA)
            pygame.draw.rect(glow, (30,110,255,22), glow.get_rect(), border_radius=14)
            surf.blit(glow, (sx-L//2-11, sy-CW//2-11))

    def step_spline(self):
        if not self.path or self.path_idx >= len(self.path):
            self.on_spline   = False
            self.lane        = self.target_lane
            self.path        = []
            self.path_idx    = 0
            self.blinker     = 0
            return True
        pt = self.path[self.path_idx]
        self.x = pt[0]; self.y = pt[1]
        self.path_idx += 1
        return False


# ──────────────────────────────────────────────────────────
#  SIMULATION
# ──────────────────────────────────────────────────────────
class Sim:

    NPC_KEYS = ["red", "lime", "gold"]

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Autonomous Driving  v3")
        self.clock  = pygame.time.Clock()
        self.font_s = pygame.font.SysFont("Consolas", 15)
        self.font_m = pygame.font.SysFont("Consolas", 19, bold=True)
        self._reset()

    # ── reset ────────────────────────────────────────────
    def _reset(self):
        self.ego = Car(260, lane_y(1), 1, "ego", EGO_SPEED, is_ego=True)

        # Spawn NPCs staggered AHEAD — they will scroll onto screen from the right
        self.npcs = []
        used_lanes = set()
        for i, key in enumerate(self.NPC_KEYS):
            candidates = [l for l in range(NUM_LANES) if l not in used_lanes]
            lane = random.choice(candidates)
            used_lanes.add(lane)
            x_start = self.ego.x + W + 60 + i * 200
            spd     = random.uniform(*NPC_SPEED_RANGE)
            self.npcs.append(Car(x_start, lane_y(lane), lane, key, spd))

        self.cam        = 0.0
        self.road_off   = 0
        self.show_sp    = True
        self.sp_preview = []
        self.n_over     = 0
        self.status     = "Cruising"
        self.warn       = False
        self.warn_alpha = 0

    # ── respawn NPC from RIGHT window edge ───────────────
    def _respawn_from_right(self, npc):
        # Find lane not crowded near entry point
        entry_x = self.ego.x + W + 80
        occupied = {
            c.lane for c in self.npcs
            if c is not npc and abs(c.x - entry_x) < 240
        }
        free = [l for l in range(NUM_LANES) if l not in occupied]
        lane = random.choice(free) if free else random.randint(0, NUM_LANES-1)

        npc.lane        = lane
        npc.target_lane = lane
        npc.y           = lane_y(lane)
        npc.x           = entry_x + random.randint(0, 80)
        npc.speed       = random.uniform(*NPC_SPEED_RANGE)
        npc.on_spline   = False
        npc.path        = []

    # ── lane-clear check ─────────────────────────────────
    def _lane_clear(self, target_lane, ego_x):
        for c in self.npcs:
            if c.lane != target_lane and c.target_lane != target_lane:
                continue
            dx = c.x - ego_x
            if -MERGE_SAFE_REAR < dx < MERGE_SAFE_FRONT:
                return False
        return True

    # ── plan B-spline lane change ─────────────────────────
    def _plan_change(self, to_lane, run=310):
        ego = self.ego
        x0, y0 = ego.x, ego.y
        y1 = lane_y(to_lane)
        dy = y1 - y0
        wps = [
            [x0,           y0],
            [x0+run*0.18,  y0+dy*0.04],
            [x0+run*0.45,  y0+dy*0.50],
            [x0+run*0.78,  y0+dy*0.96],
            [x0+run,       y1],
            [x0+run+90,    y1],
        ]
        path = build_spline(wps)
        ego.path        = path
        ego.path_idx    = 0
        ego.on_spline   = True
        ego.target_lane = to_lane
        ego.blinker     = 1 if to_lane > ego.lane else -1
        self.sp_preview = [(p[0], p[1]) for p in path]

    # ── imitation-learning policy ─────────────────────────
    def _policy(self):
        ego = self.ego
        if ego.on_spline:
            return

        # Closest NPC ahead in ego's current lane
        ahead_same = [
            c for c in self.npcs
            if abs(c.y - lane_y(ego.lane)) < LANE_H * 0.55 and c.x > ego.x
        ]
        ahead_same.sort(key=lambda c: c.x)
        leader = ahead_same[0] if ahead_same else None
        gap    = (leader.x - ego.x) if leader else 9999
        self.warn = leader is not None and gap < WARN_GAP

        if ego.state == "cruise":
            if leader and gap < OVERTAKE_GAP:
                # Build left/right options then SHUFFLE for randomized preference
                options = []
                if ego.lane > 0:
                    options.append(ego.lane - 1)   # overtake left
                if ego.lane < NUM_LANES - 1:
                    options.append(ego.lane + 1)   # overtake right
                random.shuffle(options)            # ← randomize direction

                safe = [l for l in options if self._lane_clear(l, ego.x)]

                # Prefer lane with no car immediately ahead (fewer blockers)
                def score(l):
                    blocked = [c for c in self.npcs
                               if c.lane == l and 0 < c.x - ego.x < MERGE_SAFE_FRONT]
                    return len(blocked)
                safe.sort(key=score)

                if safe:
                    tgt = safe[0]
                    ego.state = "overtaking"
                    ego.speed = EGO_SPEED + 1.6
                    self._plan_change(tgt)
                    self.n_over += 1
                    side = "Left ←" if tgt < ego.lane else "Right →"
                    self.status = f"Overtake {side}  L{tgt+1}"
                else:
                    ego.speed   = max(leader.speed + 0.1, 1.2)
                    self.status = "Waiting for gap…"

            elif leader and gap < SAFE_GAP:
                ego.speed   = max(leader.speed + 0.15, 1.5)
                self.status = "Following"
            else:
                ego.speed   = EGO_SPEED
                self.status = f"Cruising  (L{ego.lane+1})"

        elif ego.state == "overtaking":
            # Spline finished → just cruise in new lane, NO forced return
            if not ego.on_spline:
                ego.state  = "cruise"
                ego.speed  = EGO_SPEED
                self.status = f"Cruising  (L{ego.lane+1})"

    # ── update ────────────────────────────────────────────
    def update(self):
        ego = self.ego

        if ego.on_spline:
            ego.step_spline()
        else:
            ego.x += ego.speed

        for npc in self.npcs:
            npc.x += npc.speed
            # If NPC has scrolled off the LEFT (far behind ego) → re-enter from RIGHT
            if npc.x < ego.x - W * 0.55:
                self._respawn_from_right(npc)

        # NPC following avoidance (including Ego)
        all_cars = self.npcs + [ego]
        for a in self.npcs:
            closest_front = None
            min_front_gap = 9999
            for b in all_cars:
                if a is b: continue
                same_lane = False
                if b is ego:
                    if a.lane == b.lane or (b.on_spline and a.lane == b.target_lane):
                        same_lane = True
                else:
                    if a.lane == b.lane:
                        same_lane = True
                if same_lane:
                    gap_ab = b.x - a.x
                    if 0 < gap_ab < min_front_gap:
                        min_front_gap = gap_ab
                        closest_front = b
            
            if closest_front:
                if min_front_gap < 70:
                    a.speed = max(a.speed - 0.15, max(closest_front.speed - 1.0, 0.0))
                elif min_front_gap < 140:
                    a.speed = max(a.speed - 0.05, max(closest_front.speed - 0.2, 0.0))
                elif min_front_gap > 200:
                    a.speed = min(a.speed + 0.01, NPC_SPEED_RANGE[1])
            else:
                a.speed = min(a.speed + 0.01, NPC_SPEED_RANGE[1])

        self.cam      = ego.x - 260
        self.road_off = (self.road_off + int(ego.speed * 1.5)) % 120
        self._policy()

    # ──────────────────────────────────────────────────────
    #  DRAWING
    # ──────────────────────────────────────────────────────
    def _draw_road(self):
        s = self.screen
        s.fill(C_SKY)

        pygame.draw.rect(s, C_SHOULDER, (0, ROAD_TOP-28, W, 28))
        pygame.draw.rect(s, C_SHOULDER, (0, ROAD_BOT,    W, 28))

        for lane in range(NUM_LANES):
            col = C_ROAD if lane % 2 == 0 else C_ROAD_ALT
            pygame.draw.rect(s, col, (0, ROAD_TOP + lane*LANE_H, W, LANE_H))

        rng = random.Random(42)
        for _ in range(300):
            tx = rng.randint(0, W)
            ty = rng.randint(ROAD_TOP, ROAD_BOT)
            pygame.draw.circle(s, (50,52,56),
                (int((tx - self.road_off*0.3) % W), ty), 1)

        pygame.draw.rect(s, C_EDGE, (0, ROAD_TOP-4, W, 6))
        pygame.draw.rect(s, C_EDGE, (0, ROAD_BOT-2, W, 6))

        dash, gap_d = 60, 45
        period = dash + gap_d
        off    = -self.road_off % period
        for lane in range(1, NUM_LANES):
            y = ROAD_TOP + lane * LANE_H
            x = -period + off
            while x < W + period:
                x0 = max(int(x),      0)
                x1 = min(int(x+dash), W)
                if x1 > x0:
                    pygame.draw.line(s, C_DASH, (x0, y), (x1, y), 2)
                x += period

        for i in range(NUM_LANES):
            lbl = self.font_s.render(f"L{i+1}", True, (72,75,82))
            s.blit(lbl, (8, lane_y(i)-9))

    def _draw_spline_preview(self):
        if not self.show_sp or len(self.sp_preview) < 2:
            return
        pts = [(int(p[0]-self.cam), int(p[1])) for p in self.sp_preview]
        vis = [p for p in pts if -10 < p[0] < W+10]
        if len(vis) > 1:
            pygame.draw.lines(self.screen, (55,225,100), False, vis, 2)
        for i in range(0, len(pts), 45):
            if 0 <= pts[i][0] <= W:
                pygame.draw.circle(self.screen, (55,225,100), pts[i], 4)

    def _draw_warn_zone(self):
        ego = self.ego
        zx  = int(ego.x - self.cam + Car.L//2)
        zy  = int(lane_y(ego.lane) - LANE_H//2 + 4)
        if self.warn:
            self.warn_alpha = min(self.warn_alpha + 9, 105)
        else:
            self.warn_alpha = max(self.warn_alpha - 5, 0)
        col = (255,80,0, self.warn_alpha) if self.warn else (255,200,0, max(self.warn_alpha,20))
        z = pygame.Surface((WARN_GAP, LANE_H-8), pygame.SRCALPHA)
        z.fill(col)
        self.screen.blit(z, (zx, zy))

    def _draw_speedometer(self):
        s = self.screen
        cx, cy, r = W-90, H-90, 68
        pygame.draw.circle(s, (18,20,24), (cx,cy), r+4)
        pygame.draw.circle(s, (38,40,44), (cx,cy), r+4, 3)
        pygame.draw.circle(s, (13,14,18), (cx,cy), r)
        for i in range(21):
            ang = math.radians(-225 + i*(270/20))
            lf  = r-8 if i%5==0 else r-5
            x1  = cx + math.cos(ang)*(r-2)
            y1  = cy + math.sin(ang)*(r-2)
            x2  = cx + math.cos(ang)*lf
            y2  = cy + math.sin(ang)*lf
            pygame.draw.line(s, C_WHITE if i%5==0 else (80,80,80),
                             (int(x1),int(y1)), (int(x2),int(y2)), 2 if i%5==0 else 1)
        ang = math.radians(-225 + min(self.ego.speed/8.0,1)*270)
        nx = cx + math.cos(ang)*(r-14)
        ny = cy + math.sin(ang)*(r-14)
        pygame.draw.line(s, (255,80,50), (cx,cy), (int(nx),int(ny)), 3)
        pygame.draw.circle(s, (55,58,66), (cx,cy), 7)
        t1 = self.font_s.render(f"{self.ego.speed*20:.0f}", True, C_WHITE)
        t2 = self.font_s.render("km/h", True, (85,85,95))
        s.blit(t1, (cx-t1.get_width()//2, cy+16))
        s.blit(t2, (cx-t2.get_width()//2, cy+32))

    def _draw_hud(self):
        s = self.screen

        pan = pygame.Surface((265, 185), pygame.SRCALPHA)
        pan.fill((0,0,0,180))
        s.blit(pan, (10,10))
        rows = [
            ("STATUS",    self.status,               C_WHITE),
            ("EGO LANE",  f"Lane {self.ego.lane+1}", C_WHITE),
            ("EGO SPD",   f"{self.ego.speed*20:.0f} km/h", C_WHITE),
            ("NPCs",      str(len(self.npcs)),        C_WHITE),
            ("OVERTAKES", str(self.n_over),           (255,214,10)),
            ("SPLINE",    "ON" if self.show_sp else "OFF",
                           (55,225,100) if self.show_sp else (160,50,50)),
        ]
        for i,(k,v,c) in enumerate(rows):
            s.blit(self.font_s.render(f"{k:<12}", True, (110,110,125)), (18, 18+i*29))
            s.blit(self.font_s.render(v, True, c), (160, 18+i*29))

        if self.warn:
            bw = pygame.Surface((268,38), pygame.SRCALPHA)
            bw.fill((200,40,0,220))
            fcw = self.font_m.render("⚠  COLLISION WARNING", True, C_WHITE)
            bw.blit(fcw, (bw.get_width()//2-fcw.get_width()//2, 8))
            s.blit(bw, (10, 202))

        bc = {"cruise":(40,175,75),"overtaking":(225,115,0)}.get(self.ego.state, C_WHITE)
        bd = pygame.Surface((220,32), pygame.SRCALPHA)
        bd.fill((*bc,215))
        t = self.font_m.render(self.ego.state.upper(), True, C_WHITE)
        bd.blit(t, (bd.get_width()//2-t.get_width()//2, 6))
        s.blit(bd, (W//2-110, 10))

        # Mini-map
        mm_x, mm_y, mm_w, mm_h = W-315, 10, 300, 72
        mm = pygame.Surface((mm_w, mm_h), pygame.SRCALPHA)
        mm.fill((0,0,0,170))
        for i in range(1, NUM_LANES):
            pygame.draw.line(mm, (55,55,60), (0, int(i/NUM_LANES*mm_h)),
                             (mm_w, int(i/NUM_LANES*mm_h)), 1)
        view = 1200
        def mmpos(c):
            rx = (c.x - self.ego.x) / view
            ry = c.lane / NUM_LANES
            return int(mm_w//2 + rx*mm_w*0.46), int(ry*mm_h + mm_h//NUM_LANES//2)
        ex, ey = mmpos(self.ego)
        pygame.draw.rect(mm, (30,110,255), (ex-7,ey-4,14,8), border_radius=2)
        for nc in self.npcs:
            nx2, ny2 = mmpos(nc)
            if 0 < nx2 < mm_w:
                pygame.draw.rect(mm, nc.pal[0], (nx2-5,ny2-3,11,6), border_radius=2)
        pygame.draw.rect(mm, C_WHITE, mm.get_rect(), 1)
        s.blit(mm, (mm_x, mm_y))
        s.blit(self.font_s.render("MINIMAP", True, (90,90,105)), (mm_x+4, mm_y+2))

        hint = self.font_s.render("S = spline    R = reset    ESC = quit", True, (62,62,72))
        s.blit(hint, (W//2-hint.get_width()//2, H-20))

    def draw(self):
        self._draw_road()
        self._draw_warn_zone()
        self._draw_spline_preview()
        for c in sorted(self.npcs + [self.ego], key=lambda c: c.y):
            c.draw(self.screen, self.cam)
        self._draw_hud()
        self._draw_speedometer()
        pygame.display.flip()

    def run(self):
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    pygame.quit(); sys.exit()
                if ev.type == pygame.KEYDOWN:
                    if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                        pygame.quit(); sys.exit()
                    elif ev.key == pygame.K_s:
                        self.show_sp = not self.show_sp
                    elif ev.key == pygame.K_r:
                        self._reset()
            self.update()
            self.draw()
            self.clock.tick(FPS)

if __name__ == "__main__":
    Sim().run()