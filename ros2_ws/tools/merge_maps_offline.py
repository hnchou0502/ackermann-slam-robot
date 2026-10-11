#!/usr/bin/env python3
"""CM-pre 離線合圖 v2：支援 B 圖的已知初始變換（雙車「已知初始相對位姿」劇本）。

用法: cmpre_merge.py <A前綴> <B前綴> <輸出目錄> [tx ty yaw_rad]
  tx ty yaw = B 圖座標系原點在 A 圖座標系中的位姿（省略＝恆等變換）。
輸出:
  merged.pgm/.yaml  合成佔據圖
  overlay.ppm       紅=只有A牆、藍=只有B牆、黑=兩圖皆牆、白=其他
  stdout            重疊帶牆線吻合率（只算 B 也觀測過的區域，v1 的分母膨脹已修正）
"""
import sys, os, re, math

def read_yaml(path):
    d = {}
    for line in open(path):
        m = re.match(r'^(\w+):\s*(.+)$', line.strip())
        if m:
            d[m.group(1)] = m.group(2)
    res = float(d['resolution'])
    origin = [float(x) for x in d['origin'].strip('[]').split(',')[:2]]
    return res, origin

def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i+1].isspace(): i += 1
        if data[i:i+1] == b'#':
            while i < len(data) and data[i:i+1] != b'\n': i += 1
            continue
        j = i
        while j < len(data) and not data[j:j+1].isspace(): j += 1
        tokens.append(data[i:j]); i = j
    assert tokens[0] == b'P5', 'not binary pgm'
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    px = data[i+1:i+1+w*h]
    assert len(px) == w*h, f'pixel count mismatch {len(px)} vs {w*h}'
    return w, h, px

def classify(v):
    if v <= 100: return 2   # occupied
    if v >= 220: return 1   # free
    return 0                # unknown

def main(pa, pb, outdir, tf):
    tx, ty, yaw = tf
    c, s = math.cos(yaw), math.sin(yaw)
    resA, oA = read_yaml(pa + '.yaml'); wA, hA, pxA = read_pgm(pa + '.pgm')
    resB, oB = read_yaml(pb + '.yaml'); wB, hB, pxB = read_pgm(pb + '.pgm')
    assert abs(resA - resB) < 1e-9, 'resolution mismatch'
    res = resA

    # 範圍：A 的角 ∪ B 的角（B 角先變換到 A 系）
    cornersA = [(oA[0], oA[1]), (oA[0]+wA*res, oA[1]), (oA[0], oA[1]+hA*res), (oA[0]+wA*res, oA[1]+hA*res)]
    cornersB = []
    for xb, yb in [(oB[0], oB[1]), (oB[0]+wB*res, oB[1]), (oB[0], oB[1]+hB*res), (oB[0]+wB*res, oB[1]+hB*res)]:
        cornersB.append((tx + c*xb - s*yb, ty + s*xb + c*yb))
    xs = [p[0] for p in cornersA + cornersB]; ys = [p[1] for p in cornersA + cornersB]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    W = int(math.ceil((maxx - minx) / res)); H = int(math.ceil((maxy - miny) / res))
    print(f'A: {wA}x{hA}; B: {wB}x{hB}; tf=({tx:.3f},{ty:.3f},{math.degrees(yaw):.1f}deg); merged {W}x{H}')

    def atA(x, y):
        cj = int((x - oA[0]) / res); ci = hA - 1 - int((y - oA[1]) / res)
        if 0 <= ci < hA and 0 <= cj < wA: return classify(pxA[ci*wA + cj])
        return 0
    def atB(x, y):  # x,y 為 A 系座標 → 轉回 B 系再取格
        dx, dy = x - tx, y - ty
        xb = c*dx + s*dy; yb = -s*dx + c*dy
        cj = int((xb - oB[0]) / res); ci = hB - 1 - int((yb - oB[1]) / res)
        if 0 <= ci < hB and 0 <= cj < wB: return classify(pxB[ci*wB + cj])
        return 0

    merged = bytearray(W*H); overlay = bytearray(b'\xff' * (W*H*3))
    CA = [[0]*W for _ in range(H)]; CB = [[0]*W for _ in range(H)]
    for i in range(H):
        for j in range(W):
            x = minx + (j + 0.5)*res; y = maxy - (i + 0.5)*res
            ca, cb = atA(x, y), atB(x, y)
            CA[i][j], CB[i][j] = ca, cb
            k = i*W + j
            if ca == 2 or cb == 2:  merged[k] = 0
            elif ca == 1 or cb == 1: merged[k] = 254
            else:                    merged[k] = 205
            if ca == 2 and cb == 2: overlay[3*k:3*k+3] = b'\x00\x00\x00'
            elif ca == 2:           overlay[3*k:3*k+3] = b'\xdd\x00\x00'
            elif cb == 2:           overlay[3*k:3*k+3] = b'\x00\x00\xdd'

    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, 'merged.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H)); f.write(bytes(merged))
    with open(os.path.join(outdir, 'merged.yaml'), 'w') as f:
        f.write(f'image: merged.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{minx}, {miny}, 0]\nnegate: 0\n'
                f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    with open(os.path.join(outdir, 'overlay.ppm'), 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (W, H)); f.write(bytes(overlay))

    # 吻合率：只算「B 也觀測過（±3 格內有已知格）」的 A 牆格——v1 把 B 沒看過的區域也算進分母，已修正
    def b_observed_near(i, j, k=3):
        for di in range(-k, k+1):
            for dj in range(-k, k+1):
                ii, jj = i+di, j+dj
                if 0 <= ii < H and 0 <= jj < W and CB[ii][jj] != 0: return True
        return False
    b_occ = {(i, j) for i in range(H) for j in range(W) if CB[i][j] == 2}
    a_in_overlap = [(i, j) for i in range(H) for j in range(W) if CA[i][j] == 2 and b_observed_near(i, j)]
    n = len(a_in_overlap)
    print(f'重疊帶內 A 牆格數: {n}（A 牆格總數 {sum(r.count(2) for r in CA)}）')
    if n and b_occ:
        for k in (0, 1, 2, 3):
            hit = 0
            for ci, cj in a_in_overlap:
                found = False
                for di in range(-k, k+1):
                    if found: break
                    for dj in range(-k, k+1):
                        if (ci+di, cj+dj) in b_occ: found = True; break
                hit += found
            print(f'A牆格在B牆格 {k} 格({k*res*100:.0f}cm)內: {hit}/{n} = {100*hit/n:.1f}%')
    print('OK merged ->', outdir)

if __name__ == '__main__':
    tf = (0.0, 0.0, 0.0)
    if len(sys.argv) >= 7:
        tf = (float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]))
    main(sys.argv[1], sys.argv[2], sys.argv[3], tf)
