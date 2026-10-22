import numpy as np
W, H = 640, 480
LEFT_CLASSES = ["fist", "three_fingers", "thumbs_up", "thumbs_down", "victory", "open_palm", "other"]
RIGHT_CLASSES = ["pointing_up", "pinch", "open_palm", "other"]
COLS = [f"{a}{i}" for i in range(21) for a in "xyz"]
XCOLS = [c for c in COLS if c.startswith("x")]

def norm_landmarks(lm):
    """Same normalisation as scripts/record_landmarks.py: wrist at origin, scaled by wrist->middle-knuckle length."""
    p = np.array([[q.x * W, q.y * H, q.z * W] for q in lm], dtype=float)
    p -= p[0]
    return (p / np.linalg.norm(p[9][:2])).flatten()

class Softmax:
    """Multinomial logistic regression, evaluated with numpy only."""
    def __init__(self, coef, intercept, classes):
        self.coef, self.intercept, self.classes = np.asarray(coef), np.asarray(intercept), np.asarray(classes)
    def proba(self, x):
        z = np.asarray(x) @ self.coef.T + self.intercept
        z = z - z.max(axis=-1, keepdims=True)
        e = np.exp(z)
        return e / e.sum(axis=-1, keepdims=True)
