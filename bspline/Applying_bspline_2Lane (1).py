import sys, math, random
import numpy as np
import pygame

W, H      = 1200, 680
FPS       = 60
NUM_LANES = 2
LANE_H    = 140
ROAD_TOP  = (H - NUM_LANES * LANE_H) // 2
ROAD_BOT  = ROAD_TOP + NUM_LANES * LANE_H

C_SKY      = (12,  18,  28)
C_ROAD     = (42,  44,  48)
C_ROAD_ALT = (36,  38,  42)
C_EDGE     = (255, 200,   0)
C_DASH     = (200, 200, 200)
C_SHOULDER = (65,  60,  50)
C_WHITE    = (255, 255, 255)

PALETTES = {
    "ego":    ((30,  110, 255), (10,   60, 180), (120, 200, 255)),
    "red":    ((215,  45,  45), (130,  10,  10), (255, 140, 140)),
    "lime":   ((40,  190,  80), (15,  110,  35), (140, 255, 160)),
    "gold":   ((230, 170,   0), (140,  95,   0), (255, 230, 100)),
    "purple": ((160,  60, 220), ( 80,  20, 130), (210, 150, 255)),
    "teal":   ((20,  180, 180), ( 10,  100, 100), (120, 240, 240)),
    "orange": ((240, 110,  20), (150,  60,   5), (255, 180, 100)),
    "pink":   ((220,  80, 160), (120,  20,  80), (255, 180, 220)),
}
NPC_PALETTE_KEYS = ["red", "lime", "gold", "purple", "teal", "orange", "pink"]

EGO_SPEED        = 4.5
EGO_OVERTAKE_SPD = 8.0
NPC_SPEED_MIN    = 2.0
NPC_SPEED_MAX    = 3.2

NUM_NPCS        = 2
SPAWN_AHEAD_MIN = 800
SPAWN_SPACING   = 1200
RESPAWN_AHEAD   = 1600

WARN_GAP      = 140
SAFE_GAP      = 80
EMERGENCY_GAP = 40
OVERTAKE_GAP  = 120

REAR_WARN_GAP  = 160
TTC_REAR_WARN  = 45
TTC_REAR_EVADE = 30
TTC_REAR_EMERG = 20

LC_REAR_MIN  = 120
LC_FRONT_MIN = 100
TTC_LC_PAUSE = 30
TTC_LC_SLOW  = 50

SIDE_WARN_X = 60
SIDE_CRIT_X = 40

# ── Braking constants ────────────────────────────────────
BRAKE_DECEL_SOFT  = 0.12   # gentle slow-down rate (px/frame²)
BRAKE_DECEL_HARD  = 0.30   # hard braking rate
BRAKE_DECEL_EMERG = 0.60   # emergency full stop rate

TC_FRONT = (255,  80,   0)
TC_REAR  = (180,   0, 255)
TC_LEFT  = (255, 220,   0)
TC_RIGHT = (  0, 220, 255)
TC_MERGE = (255,  40, 160)
TC_BRAKE = (255,  30,  30)   # brake zone colour


def lane_y(idx):
    return ROAD_TOP + idx * LANE_H + LANE_H // 2

def catmull_rom(p0, p1, p2, p3, n=60):
    p0, p1, p2, p3 = map(np.asarray, [p0, p1, p2, p3])
    pts = []
    for i in range(n):
        t  = i / (n - 1); t2 = t*t; t3 = t2*t
        pt = 0.5*(2*p1 + (-p0+p2)*t + (2*p0-5*p1+4*p2-p3)*t2
                  + (-p0+3*p1-3*p2+p3)*t3)
        pts.append(pt.tolist())
    return pts

def build_spline(wps, n=55):
    wp = [wps[0]] + list(wps) + [wps[-1]]
    path = []
    for i in range(1, len(wp) - 2):
        seg = catmull_rom(wp[i-1], wp[i], wp[i+1], wp[i+2], n)
        if i > 1: seg = seg[1:]
        path.extend(seg)
    return path

def ttc(gap, ego_spd, other_spd):
    closing = other_spd - ego_spd
    return (gap / closing) if closing > 0.01 else 9999


class Car:
    L  = 72
    CW = 40

    def __init__(self, x, y, lane, palette_key, speed=EGO_SPEED, is_ego=False):
        self.x             = float(x)
        self.y             = float(y)
        self.lane          = lane
        self.pal           = PALETTES[palette_key]
        self.speed         = speed
        self.is_ego        = is_ego
        self.state         = "cruise"
        self.target_lane   = lane
        self.path          = []
        self.path_idx      = 0
        self.on_spline     = False
        self.spline_pause  = False
        self.blinker       = 0
        self.blink_t       = 0
        self.overtake_target = None
        # Braking state
        self.braking       = False     # True when actively braking
        self.brake_intensity = 0.0    # 0.0 → 1.0  (used for visual glow)

    @property
    def rect(self):
        return pygame.Rect(self.x - self.L//2, self.y - self.CW//2, self.L, self.CW)

    def draw(self, surf, cam):
        sx = int(self.x - cam); sy = int(self.y)
        L, CW = self.L, self.CW
        body, dark, _ = self.pal

        angle = 0
        if self.on_spline and self.path and self.path_idx < len(self.path):
            idx = self.path_idx
            lookahead = min(idx + 5, len(self.path) - 1)
            dx = self.path[lookahead][0] - self.path[idx][0]
            dy = self.path[lookahead][1] - self.path[idx][1]
            if dx != 0:
                angle = -math.degrees(math.atan2(dy, dx)) * 0.25

        car_surf = pygame.Surface((L + 60, CW + 60), pygame.SRCALPHA)
        cx, cy = (L + 60) // 2, (CW + 60) // 2

        # ── Shadow ──────────────────────────────────────
        shad = pygame.Surface((L+6, CW+6), pygame.SRCALPHA)
        pygame.draw.rect(shad, (0,0,0,55), shad.get_rect().inflate(-2,-2), border_radius=6)
        car_surf.blit(shad, (cx-L//2-1, cy-CW//2+6))

        # ── Body ────────────────────────────────────────
        br = pygame.Rect(cx-L//2, cy-CW//2, L, CW)
        pygame.draw.rect(car_surf, body, br, border_radius=9)
        pygame.draw.rect(car_surf, dark, br, 2, border_radius=9)

        # Rear window / windshield / roof
        pygame.draw.rect(car_surf, dark, (cx+L//2-24, cy-CW//2+5, 20, CW-10), border_radius=5)
        ws  = pygame.Rect(cx+L//2-46, cy-CW//2+6, 24, CW-12)
        wss = pygame.Surface((ws.w, ws.h), pygame.SRCALPHA)
        wss.fill((140,200,255,160)); car_surf.blit(wss, (ws.x, ws.y))
        pygame.draw.rect(car_surf, dark, ws, 1, border_radius=3)
        rf  = pygame.Rect(cx-L//2+16, cy-CW//2+7, L-42, CW-14)
        rfs = pygame.Surface((rf.w, rf.h), pygame.SRCALPHA)
        rfs.fill((*dark, 145)); car_surf.blit(rfs, (rf.x, rf.y))
        rw  = pygame.Rect(cx-L//2+4, cy-CW//2+6, 16, CW-12)
        rws = pygame.Surface((rw.w, rw.h), pygame.SRCALPHA)
        rws.fill((100,160,220,120)); car_surf.blit(rws, (rw.x, rw.y))

        # Wheels
        wl, ww = 16, 10
        for wx, wy in [
            (cx-L//2+6,    cy-CW//2-ww+3),
            (cx+L//2-6-wl, cy-CW//2-ww+3),
            (cx-L//2+6,    cy+CW//2-3),
            (cx+L//2-6-wl, cy+CW//2-3),
        ]:
            pygame.draw.rect(car_surf, (18,18,18), (wx,wy,wl,ww), border_radius=3)
            pygame.draw.rect(car_surf, (80,80,90), (wx+3,wy+2,wl-6,ww-4), border_radius=2)

        # Headlights
        hl_col = (255,255,180) if self.is_ego else (255,190,110)
        for hy in (cy-CW//2+6, cy+CW//2-6):
            pygame.draw.ellipse(car_surf, hl_col, (cx+L//2-7,hy-4,10,8))

        # ── Taillights — brighten when braking ──────────
        if self.is_ego and self.braking:
            # Glowing red brake lights
            bi = self.brake_intensity
            tl_col = (255, int(20*(1-bi)), int(20*(1-bi)))
            for hy in (cy-CW//2+6, cy+CW//2-6):
                pygame.draw.ellipse(car_surf, tl_col, (cx-L//2+2,hy-4,14,10))
                # outer glow halo
                gl = pygame.Surface((28, 22), pygame.SRCALPHA)
                pygame.draw.ellipse(gl, (*tl_col, int(120*bi)), (0,0,28,22))
                car_surf.blit(gl, (cx-L//2+2-7, hy-4-6))
        else:
            for hy in (cy-CW//2+6, cy+CW//2-6):
                pygame.draw.ellipse(car_surf, (200,40,40), (cx-L//2+2,hy-4,10,8))

        # Blinker
        self.blink_t = (self.blink_t+1) % 40
        if self.blinker != 0 and self.blink_t < 20:
            by = (cy-CW//2-6) if self.blinker == -1 else (cy+CW//2-4)
            pygame.draw.rect(car_surf, (255,160,0), (cx+L//2-28,by,22,6), border_radius=2)

        # Ego glow
        if self.is_ego:
            gc = (255, 50, 50) if self.braking else (30, 110, 255)
            ga = int(30 + 40 * self.brake_intensity) if self.braking else 22
            glow = pygame.Surface((L+22,CW+22), pygame.SRCALPHA)
            pygame.draw.rect(glow, (*gc, ga), glow.get_rect(), border_radius=14)
            car_surf.blit(glow, (cx-L//2-11, cy-CW//2-11))

        if self.is_ego and self.state == "passing":
            glow2 = pygame.Surface((L+30, CW+30), pygame.SRCALPHA)
            pygame.draw.rect(glow2, (255, 140, 0, 35), glow2.get_rect(), border_radius=16)
            car_surf.blit(glow2, (cx-L//2-15, cy-CW//2-15))

        if angle != 0:
            rotated_surf = pygame.transform.rotate(car_surf, angle)
            new_rect = rotated_surf.get_rect(center=(sx, sy))
            surf.blit(rotated_surf, new_rect.topleft)
        else:
            surf.blit(car_surf, (sx - cx, sy - cy))

    def step_spline(self):
        if not self.path or self.path_idx >= len(self.path):
            self.on_spline=False; self.spline_pause=False
            self.lane=self.target_lane; self.path=[]; self.path_idx=0; self.blinker=0
            return True
        if self.spline_pause: return False
        pt = self.path[self.path_idx]
        self.x=pt[0]; self.y=pt[1]; self.path_idx+=1
        return False

class Sim:

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("Autonomous Driving  v9  |  Brake Edition")
        self.clock  = pygame.time.Clock()
        self.font_s = pygame.font.SysFont("Consolas", 15)
        self.font_m = pygame.font.SysFont("Consolas", 19, bold=True)
        self._reset()

    def _reset(self):
        ego_lane = NUM_LANES // 2
        self.ego = Car(260, lane_y(ego_lane), ego_lane, "ego", EGO_SPEED, is_ego=True)

        self.npcs = []
        x_cursor  = self.ego.x + SPAWN_AHEAD_MIN
        for i in range(NUM_NPCS):
            lane    = i % NUM_LANES
            key     = NPC_PALETTE_KEYS[i % len(NPC_PALETTE_KEYS)]
            x_start = x_cursor + random.randint(0, 200)
            spd     = random.uniform(NPC_SPEED_MIN, NPC_SPEED_MAX)
            self.npcs.append(Car(x_start, lane_y(lane), lane, key, spd))
            x_cursor = x_start + SPAWN_SPACING + random.randint(0, 300)

        self.cam          = 0.0
        self.road_off     = 0
        self.show_sp      = True
        self.sp_preview   = []
        self.n_over       = 0
        self.n_evade      = 0
        self.n_collisions = 0
        self.status       = "Cruising"
        self.trapped      = False
        self.braking      = False        # NEW: braking flag for HUD

        self.thr_front   = False
        self.thr_rear    = False
        self.thr_left    = False
        self.thr_right   = False
        self.thr_merge   = False
        self.rear_ttc    = 9999
        self.lc_rear_ttc = 9999

        self.alpha = {d: 0 for d in ("front","rear","left","right","merge","brake")}

    def _respawn_npc(self, npc):
        lane_counts = {l: 0 for l in range(NUM_LANES)}
        for c in self.npcs:
            if c is not npc:
                lane_counts[c.lane] = lane_counts.get(c.lane, 0) + 1
        lane = min(lane_counts, key=lane_counts.get)
        furthest_x = max((c.x for c in self.npcs if c is not npc), default=self.ego.x)
        x_start    = max(furthest_x, self.ego.x + RESPAWN_AHEAD) + random.randint(200, 500)
        npc.lane=lane; npc.target_lane=lane; npc.y=lane_y(lane)
        npc.x=float(x_start); npc.speed=random.uniform(NPC_SPEED_MIN, NPC_SPEED_MAX)
        npc.on_spline=False; npc.path=[]

    def _scan_threats(self):
        ego = self.ego; ex = ego.x

        def in_lane(c, l):
            return abs(c.y - lane_y(l)) < LANE_H * 0.45

        def x_overlap(c, extra=0):
            return abs(c.x - ex) < (Car.L + extra)

        same        = [c for c in self.npcs if in_lane(c, ego.lane)]
        ahead_same  = sorted([c for c in same if c.x >  ex], key=lambda c: c.x)
        behind_same = sorted([c for c in same if c.x <= ex], key=lambda c: -c.x)

        leader    = ahead_same[0]  if ahead_same  else None
        chaser    = behind_same[0] if behind_same else None
        front_gap = (leader.x - ex) if leader else 9999
        rear_gap  = (ex - chaser.x) if chaser else 9999

        self.thr_front = leader is not None and front_gap < WARN_GAP

        if chaser:
            self.rear_ttc = ttc(rear_gap, ego.speed, chaser.speed)
            self.thr_rear = rear_gap < REAR_WARN_GAP or self.rear_ttc < TTC_REAR_WARN
        else:
            self.rear_ttc = 9999; self.thr_rear = False

        left_lane = ego.lane - 1
        if left_lane >= 0:
            left_cars     = [c for c in self.npcs if in_lane(c,left_lane) and x_overlap(c,SIDE_WARN_X)]
            self.thr_left = len(left_cars) > 0
        else:
            self.thr_left = False; left_cars = []

        right_lane = ego.lane + 1
        if right_lane < NUM_LANES:
            right_cars     = [c for c in self.npcs if in_lane(c,right_lane) and x_overlap(c,SIDE_WARN_X)]
            self.thr_right = len(right_cars) > 0
        else:
            self.thr_right = False; right_cars = []

        if ego.on_spline:
            tl      = ego.target_lane
            tl_cars = [c for c in self.npcs if abs(c.y - lane_y(tl)) < LANE_H*0.45]
            tl_rear = [c for c in tl_cars if c.x <= ex]
            self.thr_merge=False; self.lc_rear_ttc=9999
            for c in tl_rear:
                gap_r=ex-c.x; t2=ttc(gap_r,ego.speed,c.speed)
                if t2<self.lc_rear_ttc: self.lc_rear_ttc=t2
                if gap_r<LC_REAR_MIN or t2<TTC_LC_SLOW: self.thr_merge=True
            if (tl>ego.lane and self.thr_right) or (tl<ego.lane and self.thr_left):
                self.thr_merge=True
        else:
            self.thr_merge=False; self.lc_rear_ttc=9999

        return {
            "leader": leader, "chaser": chaser,
            "front_gap": front_gap, "rear_gap": rear_gap,
            "rear_ttc": self.rear_ttc,
            "left_cars": left_cars, "right_cars": right_cars,
        }

    def _lane_clear(self, target_lane, ego_x):
        for c in self.npcs:
            if abs(c.y - lane_y(target_lane)) > LANE_H * 0.45:
                continue
            dx = c.x - ego_x
            if -LC_REAR_MIN <= dx <= 0:
                r_ttc = ttc(-dx, self.ego.speed, c.speed)
                if dx > -(LC_REAR_MIN * 0.8) or r_ttc < TTC_LC_PAUSE + 10:
                    return False
            elif 0 < dx < LC_FRONT_MIN:
                return False
        return True

    def _plan_change(self, to_lane, run=220):
        ego = self.ego; x0,y0=ego.x,ego.y
        y1=lane_y(to_lane); dy=y1-y0
        wps=[[x0,y0],[x0+run*0.15,y0+dy*0.03],[x0+run*0.42,y0+dy*0.48],
             [x0+run*0.80,y0+dy*0.97],[x0+run,y1],[x0+run+80,y1]]
        path=build_spline(wps)
        ego.path=path; ego.path_idx=0; ego.on_spline=True
        ego.spline_pause=False; ego.target_lane=to_lane
        ego.blinker=1 if to_lane>ego.lane else -1
        self.sp_preview=[(p[0],p[1]) for p in path]

    # ── NEW: apply braking with the right deceleration rate ──
    def _apply_brake(self, leader, front_gap, rear_threat):
        """
        Decelerates ego toward leader's speed (or zero).
        Returns the new desired speed AND sets ego.braking / ego.brake_intensity.
        """
        ego = self.ego

        # How urgent is the situation?
        if front_gap < EMERGENCY_GAP:
            decel    = BRAKE_DECEL_EMERG
            target   = 0.0 if front_gap < Car.L * 0.75 else (leader.speed - 1.0 if leader else 0.0)
            intensity = 1.0
        elif front_gap < SAFE_GAP:
            decel    = BRAKE_DECEL_HARD
            target   = leader.speed - 0.3 if leader else 0.0
            intensity = 0.6 + 0.4 * (SAFE_GAP - front_gap) / max(SAFE_GAP - EMERGENCY_GAP, 1)
        else:
            # WARN_GAP range — light braking
            decel    = BRAKE_DECEL_SOFT
            t        = (front_gap - SAFE_GAP) / max(WARN_GAP - SAFE_GAP, 1)
            target   = EGO_SPEED * (0.55 + 0.45 * t)
            intensity = 0.2 + 0.3 * (1.0 - t)

        # If rear is also a threat, don't drop below the rear car's speed (avoid being rear-ended)
        if rear_threat:
            target = max(target, ego.speed - decel)

        # Smoothly decelerate toward target
        new_speed = max(ego.speed - decel, target)

        ego.braking        = True
        ego.brake_intensity = max(0.0, min(1.0, intensity))
        self.braking        = True
        return max(new_speed, 0.0)

    def _regulate_speed(self, threats):
        ego       = self.ego
        leader    = threats["leader"]
        chaser    = threats["chaser"]
        front_gap = threats["front_gap"]
        r_ttc     = threats["rear_ttc"]

        desired = EGO_OVERTAKE_SPD if ego.state == "passing" else EGO_SPEED

        if leader:
            if front_gap < WARN_GAP and ego.state != "passing":
                # Normal proportional slow-down (used when lane change IS available)
                if front_gap < EMERGENCY_GAP:
                    desired = min(desired, leader.speed - 1.2)
                    if front_gap < Car.L * 0.75: desired = 0.0
                elif front_gap < SAFE_GAP:
                    t = (front_gap - EMERGENCY_GAP) / max(SAFE_GAP - EMERGENCY_GAP, 1)
                    desired = min(desired, leader.speed + t * 1.2)
                else:
                    t = (front_gap - SAFE_GAP) / max(WARN_GAP - SAFE_GAP, 1)
                    desired = min(desired, EGO_SPEED * (0.65 + 0.35 * t))

        if chaser and self.thr_rear:
            speed_diff = max(chaser.speed - ego.speed, 0)
            urgency    = max(0, 1.0 - r_ttc / TTC_REAR_WARN)
            boost      = speed_diff * 0.6 + urgency * 2.0
            desired    = min(desired + boost, EGO_OVERTAKE_SPD + 2.0)

        return max(desired, 0.0)

    def _spline_speed_control(self):
        ego = self.ego
        if self.thr_merge and self.lc_rear_ttc < TTC_LC_PAUSE:
            ego.spline_pause=True; ego.speed=max(ego.speed-0.15,1.0)
            self.status="⚠ LC PAUSED — rear threat"; return
        ego.spline_pause=False
        if self.thr_merge and self.lc_rear_ttc < TTC_LC_SLOW:
            slowdown=(TTC_LC_SLOW-self.lc_rear_ttc)/TTC_LC_SLOW
            ego.speed=max(ego.speed-slowdown*0.4,1.2)
            self.status=f"↕ LC Slow  TTC:{self.lc_rear_ttc:.0f}f"

    def _has_passed(self, target_car):
        if target_car is None: return True
        return self.ego.x - Car.L//2 > target_car.x + Car.L//2 + 60

    # ── POLICY ────────────────────────────────────────────
    def _policy(self):
        ego     = self.ego
        threats = self._scan_threats()
        desired = self._regulate_speed(threats)

        # Clear braking state by default; _apply_brake will set it if needed
        ego.braking        = False
        ego.brake_intensity = 0.0
        self.braking        = False

        if ego.on_spline:
            self._spline_speed_control()
            if not ego.spline_pause:
                ego.speed = min(ego.speed + 0.2, EGO_OVERTAKE_SPD) if ego.state=="passing" else desired
            return

        ego.speed = desired

        leader    = threats["leader"]
        chaser    = threats["chaser"]
        front_gap = threats["front_gap"]
        r_ttc     = threats["rear_ttc"]

        def safe_overtake_lanes():
            opts = [l for l in range(NUM_LANES) if l != ego.lane]
            if self.thr_left:  opts = [l for l in opts if l >= ego.lane]
            if self.thr_right: opts = [l for l in opts if l <= ego.lane]
            return [l for l in opts if self._lane_clear(l, ego.x)]

        # ── cruise ────────────────────────────────────────
        if ego.state == "cruise":
            self.trapped = False

            if chaser and r_ttc < TTC_REAR_EVADE:
                safe = safe_overtake_lanes()
                if safe:
                    self.n_evade += 1; ego.state="overtaking"
                    ego.overtake_target=chaser; self._plan_change(safe[0])
                    self.status=f"⚠ REAR EVADE → L{safe[0]+1}"
                else:
                    self.trapped = True
                    if leader:
                        ego.speed = self._apply_brake(leader, front_gap, True)
                    else:
                        ego.speed = min(chaser.speed + 1.0, EGO_SPEED + 3.0)
                    self.status = "⚠ TRAPPED — braking"
                return

            if leader and front_gap < OVERTAKE_GAP:
                safe = safe_overtake_lanes()
                if safe:
                    self.n_over += 1
                    side = "Left ←" if safe[0] < ego.lane else "Right →"
                    self.status = f"Overtaking {side}  L{safe[0]+1}"
                    ego.state="overtaking"; ego.overtake_target=leader
                    self._plan_change(safe[0])
                else:
                    # ── NO LANE AVAILABLE — BRAKE ──────────────
                    self.trapped = True
                    ego.speed    = self._apply_brake(leader, front_gap, chaser is not None)

                    if front_gap < EMERGENCY_GAP:
                        self.status = "🛑 EMERGENCY BRAKE"
                    elif front_gap < SAFE_GAP:
                        self.status = "⛔ BRAKING HARD — no lane"
                    else:
                        self.status = "🔶 BRAKING — lane blocked"
                return

            if leader and front_gap < SAFE_GAP:
                self.status = "Following"
            else:
                self.status = f"Cruising  L{ego.lane+1}"

            tags = []
            if self.thr_left:  tags.append("◄L")
            if self.thr_right: tags.append("R►")
            if self.thr_rear:  tags.append(f"⬆REAR({r_ttc:.0f}f)")
            if tags: self.status += "  " + " ".join(tags)

        elif ego.state == "overtaking":
            ego.state="passing"; ego.speed=EGO_OVERTAKE_SPD
            self.status="⚡ Passing — accelerating"

        elif ego.state == "passing":
            if self._has_passed(ego.overtake_target):
                ego.state="cruise"; ego.overtake_target=None
                ego.speed=EGO_SPEED; self.status=f"✓ Passed — staying  L{ego.lane+1}"
            else:
                if leader and front_gap < SAFE_GAP:
                    ego.speed=max(desired, leader.speed+0.2)
                    self.status="⚡ Passing (new lane slow)"
                else:
                    ego.speed=min(ego.speed+0.1, EGO_OVERTAKE_SPD)
                    self.status=f"⚡ Passing  L{ego.lane+1}"

    def update(self):
        ego = self.ego
        if ego.on_spline:
            done = ego.step_spline()
            if done and ego.state == "overtaking":
                self._policy()
        else:
            ego.x += ego.speed

        for npc in self.npcs:
            npc.x += npc.speed
            if npc.x < ego.x - W * 0.7:
                self._respawn_npc(npc)
            if ego.rect.colliderect(npc.rect):
                if not getattr(npc, 'collided', False):
                    self.n_collisions += 1; npc.collided = True
            else:
                npc.collided = False

        all_cars = self.npcs + [ego]
        for a in self.npcs:
            min_front_gap = 9999; closest_front = None
            for b in all_cars:
                if a is b: continue
                same_lane = (a.lane==b.lane) or (b is ego and b.on_spline and a.lane==b.target_lane)
                if same_lane:
                    gap_ab = b.x - a.x
                    if 0 < gap_ab < min_front_gap:
                        min_front_gap=gap_ab; closest_front=b
            if closest_front:
                if min_front_gap < 80:
                    a.speed=max(a.speed-0.15, max(closest_front.speed-1.0, 0.0))
                elif min_front_gap < 160:
                    a.speed=max(a.speed-0.08, max(closest_front.speed-0.2, 0.0))
                elif min_front_gap > 250:
                    a.speed=min(a.speed+0.02, NPC_SPEED_MAX)
            else:
                a.speed=min(a.speed+0.02, NPC_SPEED_MAX)

        self.cam      = ego.x - 260
        self.road_off = (self.road_off + int(ego.speed * 1.5)) % 120
        self._policy()

    
    def _draw_road(self):
        s = self.screen
        s.fill(C_SKY)
        pygame.draw.rect(s, C_SHOULDER, (0, ROAD_TOP-30, W, 30))
        pygame.draw.rect(s, C_SHOULDER, (0, ROAD_BOT,    W, 30))
        for lane in range(NUM_LANES):
            col = C_ROAD if lane%2==0 else C_ROAD_ALT
            pygame.draw.rect(s, col, (0, ROAD_TOP+lane*LANE_H, W, LANE_H))
        rng = random.Random(42)
        for _ in range(240):
            tx=rng.randint(0,W); ty=rng.randint(ROAD_TOP,ROAD_BOT)
            pygame.draw.circle(s,(50,52,56),(int((tx-self.road_off*0.3)%W),ty),1)
        pygame.draw.rect(s,C_EDGE,(0,ROAD_TOP-4,W,6))
        pygame.draw.rect(s,C_EDGE,(0,ROAD_BOT-2,W,6))
        dash,gap_d=55,45; period=dash+gap_d
        off=-self.road_off%period
        for lane_div in range(1,NUM_LANES):
            y=ROAD_TOP+lane_div*LANE_H; x=-period+off
            while x<W+period:
                x0=max(int(x),0); x1=min(int(x+dash),W)
                if x1>x0: pygame.draw.line(s,C_DASH,(x0,y),(x1,y),2)
                x+=period
        for i in range(NUM_LANES):
            lbl=self.font_s.render(f"L{i+1}",True,(72,75,82))
            s.blit(lbl,(8,lane_y(i)-9))

    def _smooth_alpha(self, key, active, rate_up=10, rate_dn=6, cap=100):
        if active: self.alpha[key]=min(self.alpha[key]+rate_up,cap)
        else:      self.alpha[key]=max(self.alpha[key]-rate_dn,0)
        return self.alpha[key]

    def _draw_threat_zones(self):
        ego=self.ego; ex_s=int(ego.x-self.cam)

        a=self._smooth_alpha("front",self.thr_front)
        if a>0:
            zy=int(lane_y(ego.lane)-LANE_H//2+4)
            z=pygame.Surface((WARN_GAP,LANE_H-8),pygame.SRCALPHA)
            z.fill((*TC_FRONT,a)); self.screen.blit(z,(ex_s+Car.L//2,zy))

        a=self._smooth_alpha("rear",self.thr_rear)
        if a>0:
            zy=int(lane_y(ego.lane)-LANE_H//2+4)
            urg=max(0,1.0-self.rear_ttc/TTC_REAR_WARN) if self.rear_ttc<TTC_REAR_WARN else 0
            zw=int(REAR_WARN_GAP*(0.4+0.6*urg)); rx=ex_s-Car.L//2-zw
            rz=pygame.Surface((zw,LANE_H-8),pygame.SRCALPHA)
            rz.fill((*TC_REAR,a)); self.screen.blit(rz,(rx,zy))
            lbl=f"◄ REAR TTC:{self.rear_ttc:.0f}f" if self.rear_ttc<9999 else "◄ REAR"
            self.screen.blit(self.font_s.render(lbl,True,(210,130,255)),(max(rx+4,0),zy+4))

        a=self._smooth_alpha("left",self.thr_left)
        if a>0 and ego.lane>0:
            zy=int(lane_y(ego.lane-1)-LANE_H//2+4)
            lz=pygame.Surface((Car.L+SIDE_WARN_X*2,LANE_H-8),pygame.SRCALPHA)
            lz.fill((*TC_LEFT,a)); self.screen.blit(lz,(ex_s-Car.L//2-SIDE_WARN_X,zy))
            self.screen.blit(self.font_s.render("◄ LEFT",True,(255,230,80)),(ex_s-28,zy+4))

        a=self._smooth_alpha("right",self.thr_right)
        if a>0 and ego.lane<NUM_LANES-1:
            zy=int(lane_y(ego.lane+1)-LANE_H//2+4)
            rz=pygame.Surface((Car.L+SIDE_WARN_X*2,LANE_H-8),pygame.SRCALPHA)
            rz.fill((*TC_RIGHT,a)); self.screen.blit(rz,(ex_s-Car.L//2-SIDE_WARN_X,zy))
            self.screen.blit(self.font_s.render("RIGHT ►",True,(80,230,255)),(ex_s-28,zy+4))

        a=self._smooth_alpha("merge",self.thr_merge)
        if a>0 and ego.on_spline:
            zy=int(lane_y(ego.target_lane)-LANE_H//2+4)
            mz=pygame.Surface((Car.L+SIDE_CRIT_X*2,LANE_H-8),pygame.SRCALPHA)
            mz.fill((*TC_MERGE,a)); self.screen.blit(mz,(ex_s-Car.L//2-SIDE_CRIT_X,zy))
            lc_str=f"LC REAR TTC:{self.lc_rear_ttc:.0f}f" if self.lc_rear_ttc<9999 else "LC BLOCKED"
            self.screen.blit(self.font_s.render(lc_str,True,(255,100,200)),(ex_s-40,zy+4))

        # ── BRAKE ZONE — red expanding zone ahead of ego ──
        a = self._smooth_alpha("brake", self.braking)
        if a > 0:
            bi  = ego.brake_intensity
            zy  = int(lane_y(ego.lane) - LANE_H//2 + 4)
            zw  = int((WARN_GAP * 0.5 + WARN_GAP * 0.5 * bi))
            bz  = pygame.Surface((zw, LANE_H-8), pygame.SRCALPHA)
            for dx in range(zw):
                fade = int(a * (1 - dx/zw) * 1.2)
                pygame.draw.line(bz, (*TC_BRAKE, min(fade,200)), (dx,0),(dx,LANE_H-8))
            self.screen.blit(bz, (ex_s + Car.L//2, zy))

        if self.trapped:
            halo=pygame.Surface((Car.L+40,Car.CW+40),pygame.SRCALPHA)
            pygame.draw.rect(halo,(255,30,30,80),halo.get_rect(),border_radius=14)
            self.screen.blit(halo,(ex_s-Car.L//2-20,int(ego.y)-Car.CW//2-20))

        if ego.state=="passing":
            trail=pygame.Surface((180,Car.CW+20),pygame.SRCALPHA)
            for i in range(6):
                at=int(40*(1-i/6))
                pygame.draw.rect(trail,(255,160,0,at),(i*30,0,180-i*30,Car.CW+20),border_radius=8)
            self.screen.blit(trail,(ex_s-Car.L//2-180,int(ego.y)-Car.CW//2-10))

        if ego.overtake_target is not None:
            tx_s=int(ego.overtake_target.x-self.cam); ty_s=int(ego.overtake_target.y)
            pygame.draw.line(self.screen,(255,200,0),(ex_s+Car.L//2,int(ego.y)),(tx_s-Car.L//2,ty_s),2)
            pygame.draw.circle(self.screen,(255,200,0),(tx_s,ty_s),8,2)
            lbl=self.font_s.render("TARGET",True,(255,220,60))
            self.screen.blit(lbl,(tx_s-lbl.get_width()//2,ty_s-Car.CW//2-18))

    def _draw_spline_preview(self):
        if not self.show_sp or len(self.sp_preview)<2: return
        pts=[(int(p[0]-self.cam),int(p[1])) for p in self.sp_preview]
        vis=[p for p in pts if -10<p[0]<W+10]
        if len(vis)>1:
            col=(255,80,160) if self.ego.spline_pause else (55,225,100)
            pygame.draw.lines(self.screen,col,False,vis,2)
        for i in range(0,len(pts),45):
            if 0<=pts[i][0]<=W:
                pygame.draw.circle(self.screen,(55,225,100),pts[i],4)

    def _draw_speedometer(self):
        s=self.screen; cx,cy,r=W-90,H-90,68
        pygame.draw.circle(s,(18,20,24),(cx,cy),r+4)
        pygame.draw.circle(s,(38,40,44),(cx,cy),r+4,3)
        pygame.draw.circle(s,(13,14,18),(cx,cy),r)
        for i in range(21):
            ang=math.radians(-225+i*(270/20)); lf=r-8 if i%5==0 else r-5
            x1=cx+math.cos(ang)*(r-2); y1=cy+math.sin(ang)*(r-2)
            x2=cx+math.cos(ang)*lf;    y2=cy+math.sin(ang)*lf
            pygame.draw.line(s,C_WHITE if i%5==0 else (80,80,80),(int(x1),int(y1)),(int(x2),int(y2)),2 if i%5==0 else 1)
        max_spd=EGO_OVERTAKE_SPD+2.0
        ang=math.radians(-225+min(self.ego.speed/max_spd,1)*270)
        nx=cx+math.cos(ang)*(r-14); ny=cy+math.sin(ang)*(r-14)
        needle_col=(255,30,30) if self.braking else (255,140,0) if self.ego.state=="passing" else (255,80,50)
        pygame.draw.line(s,needle_col,(cx,cy),(int(nx),int(ny)),3)
        pygame.draw.circle(s,(55,58,66),(cx,cy),7)
        t1=self.font_s.render(f"{self.ego.speed*20:.0f}",True,C_WHITE)
        t2=self.font_s.render("km/h",True,(85,85,95))
        s.blit(t1,(cx-t1.get_width()//2,cy+16))
        s.blit(t2,(cx-t2.get_width()//2,cy+32))

    def _draw_hud(self):
        s=self.screen
        pan=pygame.Surface((320,230),pygame.SRCALPHA)
        pan.fill((0,0,0,190)); s.blit(pan,(10,10))

        state_col={
            "cruise":    (40,175,75),
            "overtaking":(225,115,0),
            "passing":   (255,140,0),
        }.get(self.ego.state,C_WHITE)

        # Override state colour to red when braking
        if self.braking:
            state_col = (230, 40, 40)

        ttc_str=f"{self.rear_ttc:.0f} f" if self.rear_ttc<9999 else "clear"
        ttc_col=((255,80,80) if self.rear_ttc<TTC_REAR_EVADE else
                 (255,200,80) if self.rear_ttc<TTC_REAR_WARN else (80,220,80))
        lc_str=f"{self.lc_rear_ttc:.0f} f" if self.lc_rear_ttc<9999 else "—"
        lc_col=(255,80,80) if self.lc_rear_ttc<TTC_LC_PAUSE else \
               (255,200,80) if self.lc_rear_ttc<TTC_LC_SLOW else (80,220,80)
        tgt_str=f"L{self.ego.overtake_target.lane+1}" if self.ego.overtake_target else "none"
        brake_str=f"{self.ego.brake_intensity*100:.0f}%" if self.braking else "off"
        brake_col=(255,60,60) if self.braking else (80,180,80)

        rows=[
            ("STATUS",   self.status,                    C_WHITE),
            ("STATE",    self.ego.state.upper(),          state_col),
            ("EGO LANE", f"L{self.ego.lane+1}",           C_WHITE),
            ("SPEED",    f"{self.ego.speed*20:.0f} km/h", C_WHITE),
            ("REAR TTC", ttc_str,                         ttc_col),
            ("LC TTC",   lc_str,                          lc_col),
            ("TARGET",   tgt_str,                         (255,220,60)),
            ("BRAKING",  brake_str,                       brake_col),
            ("TRAPPED",  "YES ⚠" if self.trapped else "no",
                         (255,60,60) if self.trapped else (80,180,80)),
        ]
        for i,(k,v,c) in enumerate(rows):
            s.blit(self.font_s.render(f"{k:<11}",True,(110,110,125)),(18,18+i*22))
            s.blit(self.font_s.render(v,True,c),(165,18+i*22))

        # Threat pills
        indicators=[
            ("FWD",   self.thr_front, TC_FRONT),
            ("REAR",  self.thr_rear,  TC_REAR),
            ("LEFT",  self.thr_left,  TC_LEFT),
            ("RIGHT", self.thr_right, TC_RIGHT),
            ("MERGE", self.thr_merge, TC_MERGE),
            ("BRAKE", self.braking,   TC_BRAKE),
        ]
        ix=18
        for label,active,col in indicators:
            bg=pygame.Surface((46,22),pygame.SRCALPHA)
            bg.fill((*col,220 if active else 50)); s.blit(bg,(ix,222))
            s.blit(self.font_s.render(label,True,C_WHITE if active else (90,90,90)),(ix+3,224))
            ix+=50

        # Warning banners
        by=248
        if self.thr_front:
            self._banner(s,"⚠ FRONT COLLISION RISK",(200,40,0),by); by+=36
        if self.thr_rear:
            uc=(255,0,80) if self.rear_ttc<TTC_REAR_EVADE else (130,0,200)
            self._banner(s,f"◄ REAR TTC:{self.rear_ttc:.0f}f",uc,by); by+=36
        if self.thr_left:
            self._banner(s,"◄ LEFT SIDE BLOCKED",(180,160,0),by); by+=36
        if self.thr_right:
            self._banner(s,"► RIGHT SIDE BLOCKED",(0,160,200),by); by+=36
        if self.thr_merge:
            self._banner(s,f"! LC THREAT TTC:{self.lc_rear_ttc:.0f}f",(200,20,130),by); by+=36

    
        if self.braking:
            bi=self.ego.brake_intensity
            if bi > 0.85:
                self._banner(s, "🛑  EMERGENCY BRAKE", (220, 20, 20), by); by+=36
            elif bi > 0.5:
                self._banner(s, "⛔  BRAKING HARD", (190, 40, 0), by); by+=36
            else:
                self._banner(s, "🔶  BRAKING — lane blocked", (160, 100, 0), by); by+=36

        if self.trapped and not self.braking:
            self._banner(s,"⚠ TRAPPED — SPEED BALANCED",(160,20,20),by)

        # Counter strip
        cnt=self.font_s.render(
            f"✓ OVT:{self.n_over}  EVD:{self.n_evade}  💥:{self.n_collisions}",
            True,(150,200,150) if self.n_collisions==0 else (255,100,100))
        s.blit(cnt,(10,H-38))

        # State badge
        bd=pygame.Surface((280,32),pygame.SRCALPHA); bd.fill((*state_col,210))
        t=self.font_m.render(self.ego.state.upper(),True,C_WHITE)
        bd.blit(t,(bd.get_width()//2-t.get_width()//2,6)); s.blit(bd,(W//2-140,10))

        # Mini-map
        mm_x,mm_y,mm_w,mm_h=W-315,10,300,72
        mm=pygame.Surface((mm_w,mm_h),pygame.SRCALPHA); mm.fill((0,0,0,170))
        for i in range(1,NUM_LANES):
            pygame.draw.line(mm,(55,55,60),(0,int(i/NUM_LANES*mm_h)),(mm_w,int(i/NUM_LANES*mm_h)),1)
        view=1800
        def mmpos(c):
            rx=(c.x-self.ego.x)/view; ry=c.lane/NUM_LANES
            return (int(mm_w//2+rx*mm_w*0.46),int(ry*mm_h+mm_h//NUM_LANES//2))
        ex2,ey2=mmpos(self.ego)
        ego_dot_col=(255,60,60) if self.braking else (30,110,255)
        pygame.draw.rect(mm,ego_dot_col,(ex2-7,ey2-4,14,8),border_radius=2)
        for nc in self.npcs:
            nx2,ny2=mmpos(nc)
            if 0<nx2<mm_w:
                pygame.draw.rect(mm,nc.pal[0],(nx2-5,ny2-3,11,6),border_radius=2)
        pygame.draw.rect(mm,C_WHITE,mm.get_rect(),1)
        s.blit(mm,(mm_x,mm_y))
        s.blit(self.font_s.render("MINIMAP",True,(90,90,105)),(mm_x+4,mm_y+2))

        hint=self.font_s.render("S = spline    R = reset    ESC = quit",True,(62,62,72))
        s.blit(hint,(W//2-hint.get_width()//2,H-18))

    def _banner(self,surf,text,colour,y):
        bw=pygame.Surface((315,34),pygame.SRCALPHA); bw.fill((*colour,210))
        t=self.font_m.render(text,True,C_WHITE)
        bw.blit(t,(bw.get_width()//2-t.get_width()//2,6)); surf.blit(bw,(10,y))

    def draw(self):
        self._draw_road()
        self._draw_threat_zones()
        self._draw_spline_preview()
        for c in sorted(self.npcs+[self.ego],key=lambda c:c.y):
            c.draw(self.screen,self.cam)
        self._draw_hud()
        self._draw_speedometer()
        pygame.display.flip()

    def run(self):
        while True:
            for ev in pygame.event.get():
                if ev.type==pygame.QUIT: pygame.quit(); sys.exit()
                if ev.type==pygame.KEYDOWN:
                    if ev.key in (pygame.K_ESCAPE,pygame.K_q): pygame.quit(); sys.exit()
                    elif ev.key==pygame.K_s: self.show_sp=not self.show_sp
                    elif ev.key==pygame.K_r: self._reset()
            self.update(); self.draw()
            self.clock.tick(FPS)


if __name__ == "__main__":
    Sim().run()