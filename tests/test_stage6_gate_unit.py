#!/usr/bin/env python3
"""阶段 6 门禁运动学纯函数单元测试。

对抗审查后补充：此前的集成测试从未覆盖 vx>max_vx 且 wz≠0 的输入（遥控形态，
twist_mux 优先级 90 可直达门禁），恰好漏掉"vx 截幅在半径裁剪之后"的顺序 bug
（旧实现输出半径可跌破底盘 min_turn_radius=0.4764，触发驱动 SPINNING 模式切换）。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "ws_nav/src/ranger_navigation"))

from ranger_navigation.motion_gate import enforce_kinematics

P = dict(max_vx=0.20, max_wz=0.60, min_turn_radius=0.55, vx_turn_min=0.05)
RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond)))
    print(('PASS ' if cond else 'FAIL ') + name + (f' | {detail}' if detail else ''))


def run(vx, vy, wz):
    return enforce_kinematics(vx, vy, wz, **P)


def test_cases():
    cases = [
        # (vx, vy, wz) -> 期望 (vx, wz, flags)
        ((0.15, 0.0, 0.0), (0.15, 0.0), []),                    # 直行原样通过
        ((0.15, 0.0, 0.25), (0.15, 0.25), []),                  # R=0.6 合法通过
        ((0.15, 0.01, 0.0), (0.0, 0.0), ['lateral_rejected']),  # 横移：整条归零
        ((-0.10, 0.0, 0.0), (0.0, 0.0), ['reverse_clamped']),   # 倒车钳零
        ((0.0, 0.0, 0.5), (0.0, 0.0), ['spin_prevented']),      # 纯旋转禁
        ((0.04, 0.0, 0.4), (0.04, 0.0), ['spin_prevented']),    # 低速转向禁
        ((0.30, 0.0, 0.0), (0.20, 0.0), ['vx_capped']),         # 限速
        ((0.15, 0.0, 0.9), (0.15, 0.15 / 0.55), ['wz_capped', 'radius_clipped']),
        # 角速度限幅后半径仍不足 -> 再裁半径（顺序：先限幅后裁半径）
        ((float('nan'), 0.0, 0.0), (0.0, 0.0), ['invalid']),    # 非有限值
    ]
    for inp, exp, expf in cases:
        vx, vy, wz, fl = run(*inp)
        check(f'case{inp}', abs(vx - exp[0]) < 1e-9 and abs(wz - exp[1]) < 1e-9
              and fl == expf, f'-> ({vx:.3f},{wz:.3f}) {fl}')


def test_regression_order():
    """审查确认的顺序 bug：必须先限幅 vx 再裁半径，输出半径 >= 0.55。"""
    vx, vy, wz, fl = run(0.25, 0.0, 0.5)
    check('regression_cap_before_radius',
          abs(vx - 0.20) < 1e-9 and abs(wz - 0.20 / 0.55) < 1e-9,
          f'out=({vx:.3f},{wz:.4f}) R={abs(vx / wz):.3f} flags={fl}')


def test_sweep_invariant():
    """网格扫描不变量：任意输入下输出满足 R >= min_turn_radius-ε 或 wz==0。"""
    bad = []
    for i in range(0, 36):
        for j in range(-20, 21):
            vx, vy, wz, fl = run(i * 0.01, 0.0, j * 0.05)
            if abs(wz) > 1e-6:
                r = abs(vx) / abs(wz)
                if r < P['min_turn_radius'] - 1e-6:
                    bad.append((round(i * 0.01, 2), round(j * 0.05, 2),
                                round(vx, 3), round(wz, 3), round(r, 3)))
    check('sweep_radius_invariant', not bad, f'{len(bad)} 违例, 例: {bad[:3]}')

    bad2 = []
    for i in range(0, 36):
        for j in range(-20, 21):
            vx, vy, wz, fl = run(i * 0.01, 0.0, j * 0.05)
            if vx > P['max_vx'] + 1e-9 or abs(wz) > P['max_wz'] + 1e-9 or abs(vy) > 1e-9:
                bad2.append((i * 0.01, j * 0.05, vx, wz))
    check('sweep_bounds_invariant', not bad2, f'{len(bad2)} 越界, 例: {bad2[:3]}')


if __name__ == '__main__':
    test_cases()
    test_regression_order()
    test_sweep_invariant()
    failed = [n for n, ok in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    sys.exit(1 if failed else 0)
