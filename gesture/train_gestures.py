#!/usr/bin/env python3
"""Train one logistic-regression model per hand from data/landmarks_*.csv -> models/gesture_models.npz"""
import argparse, glob, sys
from pathlib import Path
import numpy as np, pandas as pd
from sklearn.linear_model import LogisticRegression
sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import COLS, XCOLS, LEFT_CLASSES, RIGHT_CLASSES, Softmax

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SIDE = {"s1": "Right", "s2": "Right", "s3": "Left"}   # which physical hand each session used

def load(side_map):
    parts = []
    for f in sorted(glob.glob(str(ROOT / "data" / "landmarks_*.csv"))):
        d = pd.read_csv(f)
        for s, g in d.groupby("session"):
            if s not in side_map:
                sys.exit(f"session {s} ({f}) has no hand side: pass --side {s}=Left or --side {s}=Right")
            g = g[g.hand == side_map[s]].copy()     # drop frames whose hand label disagrees with the session
            g["side"] = side_map[s]
            parts.append(g)
    return pd.concat(parts, ignore_index=True)

def left_training(d):
    L = d[(d.side == "Left") & d.label.isin(LEFT_CLASSES)]
    R = d[(d.side == "Right") & d.label.isin(LEFT_CLASSES)].copy()
    R[XCOLS] = -R[XCOLS]                              # mirror right-hand data into left-hand shape
    return pd.concat([L, R], ignore_index=True)

def right_training(d):
    return d[(d.side == "Right") & d.label.isin(RIGHT_CLASSES)]

def fit(df):
    return LogisticRegression(max_iter=3000).fit(df[COLS].values, df.label.values)

def report(title, m, te):
    pred = m.predict(te[COLS].values)
    print(f"{title}: per-frame accuracy {(pred == te.label.values).mean():.3f} ({len(te)} frames)")
    for c in m.classes_:
        sel = te.label.values == c
        print(f"    {c:13s} {(pred[sel] == c).sum():4d}/{sel.sum():4d}")

def check(d):
    rs = sorted(d[d.side == "Right"].session.unique()); ls = sorted(d[d.side == "Left"].session.unique())
    for s in ls:
        te = d[(d.session == s) & d.label.isin(LEFT_CLASSES)]
        report(f"HELD-OUT LEFT model (mirrored right sessions -> {s})", fit(left_training(d[d.side == "Right"])), te)
    for a in rs:
        for b in rs:
            if a != b:
                report(f"HELD-OUT RIGHT model ({a} -> {b})", fit(right_training(d[d.session == a])), right_training(d[d.session == b]))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="only re-run the held-out evaluations")
    ap.add_argument("--side", action="append", default=[], help="e.g. s4=Left")
    ap.add_argument("--out", default=str(ROOT / "models" / "gesture_models.npz"))
    a = ap.parse_args()
    side_map = dict(DEFAULT_SIDE); side_map.update(dict(x.split("=") for x in a.side))
    d = load(side_map)
    print("frames kept per session/side:", d.groupby(["session", "side"]).size().to_dict())
    if a.check:
        check(d); return
    out = {}
    for name, df in (("left", left_training(d)), ("right", right_training(d))):
        m = fit(df)
        sm = Softmax(m.coef_, m.intercept_, m.classes_)
        diff = float(np.abs(sm.proba(df[COLS].values) - m.predict_proba(df[COLS].values)).max())
        acc = float((m.predict(df[COLS].values) == df.label.values).mean())
        print(f"{name}: {len(df)} frames, per class {df.label.value_counts().to_dict()}")
        print(f"   numpy vs sklearn probability max difference: {diff:.2e} | accuracy on its OWN training frames {acc:.3f} (not a test)")
        out.update({f"{name}_coef": m.coef_, f"{name}_intercept": m.intercept_, f"{name}_classes": np.array([str(c) for c in m.classes_], dtype="U")})
    Path(a.out).parent.mkdir(exist_ok=True)
    np.savez(a.out, **out)
    print("saved", a.out)
    z = np.load(a.out, allow_pickle=False)           # same loading rule as gesture_node.py
    print("reload check OK:", {k: tuple(z[k].shape) for k in z.files})
    print("left classes :", list(z["left_classes"]))
    print("right classes:", list(z["right_classes"]))

if __name__ == "__main__":
    main()
