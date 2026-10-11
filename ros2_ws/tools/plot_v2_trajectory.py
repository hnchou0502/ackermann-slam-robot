#!/usr/bin/env python3
"""把 score_follow 的 CSV 畫成 SVG 軌跡圖——零第三方依賴（Pi 與 Windows 都沒有 matplotlib）。

    python3 plot_v2_trajectory.py <score.csv> <out.svg>

畫兩條軌跡（leader 藍、follower 橘）＋起點標記＋幾條等時連線。
兩條線幾乎重疊＝follower 走在 leader 走過的路上（沒有切內線），這是 V2 的核心主張。
"""
import csv
import sys


def load(path):
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    lead = [(float(r['leader_x']), float(r['leader_y'])) for r in rows]
    foll = [(float(r['follower_x']), float(r['follower_y'])) for r in rows]
    t = [float(r['t']) for r in rows]
    return t, lead, foll


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    t, lead, foll = load(sys.argv[1])

    pts = lead + foll
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    pad = 0.4
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad

    W, H = 900, int(900 * (y1 - y0) / (x1 - x0))
    sx = lambda x: (x - x0) / (x1 - x0) * W               # noqa: E731
    sy = lambda y: H - (y - y0) / (y1 - y0) * H           # noqa: E731  (y 軸朝上)

    def poly(pp):
        return ' '.join(f'{sx(x):.1f},{sy(y):.1f}' for x, y in pp)

    # 每 8 秒一條等時連線，顯示兩車的瞬時間距
    links = []
    step = 8.0
    next_t = t[0]
    for i, ti in enumerate(t):
        if ti >= next_t:
            links.append((lead[i], foll[i]))
            next_t += step

    grid = []
    gx = int(x0)
    while gx <= x1:
        grid.append(f'<line x1="{sx(gx):.1f}" y1="0" x2="{sx(gx):.1f}" y2="{H}" '
                    f'stroke="#e5e7eb" stroke-width="1"/>')
        gx += 1
    gy = int(y0)
    while gy <= y1:
        grid.append(f'<line x1="0" y1="{sy(gy):.1f}" x2="{W}" y2="{sy(gy):.1f}" '
                    f'stroke="#e5e7eb" stroke-width="1"/>')
        gy += 1

    link_svg = ''.join(
        f'<line x1="{sx(a[0]):.1f}" y1="{sy(a[1]):.1f}" '
        f'x2="{sx(b[0]):.1f}" y2="{sy(b[1]):.1f}" '
        f'stroke="#9ca3af" stroke-width="1" stroke-dasharray="4,3"/>'
        for a, b in links)

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
<rect width="{W}" height="{H}" fill="white"/>
{''.join(grid)}
{link_svg}
<polyline points="{poly(lead)}" fill="none" stroke="#2563eb" stroke-width="2.5"/>
<polyline points="{poly(foll)}" fill="none" stroke="#ea580c" stroke-width="2.5"
          stroke-dasharray="7,4"/>
<circle cx="{sx(lead[0][0]):.1f}" cy="{sy(lead[0][1]):.1f}" r="6" fill="#2563eb"/>
<circle cx="{sx(foll[0][0]):.1f}" cy="{sy(foll[0][1]):.1f}" r="6" fill="#ea580c"/>
<g font-family="sans-serif" font-size="15">
  <rect x="12" y="12" width="290" height="76" fill="white" stroke="#d1d5db"/>
  <line x1="24" y1="36" x2="60" y2="36" stroke="#2563eb" stroke-width="2.5"/>
  <text x="68" y="41" fill="#111">leader (car1) — 開迴路 8 字</text>
  <line x1="24" y1="62" x2="60" y2="62" stroke="#ea580c" stroke-width="2.5"
        stroke-dasharray="7,4"/>
  <text x="68" y="67" fill="#111">follower (car2) — 麵包屑＋純追蹤</text>
</g>
<text x="12" y="{H - 14}" font-family="sans-serif" font-size="13" fill="#6b7280">
  網格 1 m；灰虛線＝每 8 秒的兩車連線。單位 m（car2 的 odom 座標系）
</text>
</svg>
'''
    with open(sys.argv[2], 'w', encoding='utf-8') as f:
        f.write(svg)
    print(f'已輸出 {sys.argv[2]}（{W}x{H}，leader {len(lead)} 點）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
