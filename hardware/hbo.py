"""Two Gaussian processes with Augmented Expected Improvement for bounded PID trials."""
from dataclasses import dataclass
import warnings

import numpy as np
from scipy.linalg import solve_triangular
from scipy.stats import norm
from sklearn.exceptions import ConvergenceWarning
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, Matern, WhiteKernel
from threadpoolctl import threadpool_limits

BOUNDS = np.array([(0.12, 0.95), (0.025, 0.34), (0.04, 0.35)])
LOG_LOW = np.log(BOUNDS[:, 0])
LOG_SPAN = np.log(BOUNDS[:, 1]) - LOG_LOW


def unit(gains):
    return (np.log(np.asarray(gains)) - LOG_LOW)/LOG_SPAN


def gains_at(x):
    return np.exp(LOG_LOW + np.asarray(x)*LOG_SPAN)


@dataclass(frozen=True)
class Score:
    total: float
    mae: float
    overshoot: float
    undershoot: float
    steady: float
    ripple: float
    pulse_change: float
    capped_pct: float
    settle_up_s: float
    settle_down_s: float
    upper_stop: bool


def score_trial(phases, upper_stop=False):
    def segment(name):
        rows = phases.get(name, [])
        if len(rows) < 120 or rows[-1].ms - rows[0].ms < 5300:
            raise ValueError(f"Incomplete {name.lower()} segment.")
        t = np.array([r.ms-rows[0].ms for r in rows])/1000
        if np.max(np.diff(t)) > 0.24:
            raise ValueError(f"Telemetry gap in {name.lower()} segment.")
        return t, np.array([r.angle for r in rows]), np.array([r.target for r in rows]), np.array(
            [r.pulse for r in rows]), rows[0].cap

    def settling(t, e):
        bad = np.flatnonzero(np.abs(e) > 1.0)
        idx = int(bad[-1]+1) if len(bad) else 0
        return float(t[idx]) if idx < len(t) and t[-1]-t[idx] >= 0.5 else float("nan")

    if upper_stop:
        return Score(65.0, *([float("nan")]*9), True)
    tu, au, ru, pu, cap = segment("Up")
    td, ad, rd, pd, _ = segment("Down")
    eu, ed = au-ru, ad-rd
    ends = [eu[tu >= tu[-1]-2], ed[td >= td[-1]-2]]
    mae = float(np.mean(np.abs(np.r_[eu, ed])))
    overshoot, undershoot = max(0., float(np.max(eu))), max(0., -float(np.min(ed)))
    steady = float(np.mean([np.mean(np.abs(e)) for e in ends]))
    ripple = float(np.mean([np.std(e) for e in ends]))
    change = float(np.mean(np.abs(np.r_[np.diff(pu), np.diff(pd)])))
    capped = float(100*np.mean(np.r_[pu, pd] >= cap))
    total = max(0.0001, mae + 0.3*(overshoot+undershoot) + 1.5*steady + 0.6*ripple + 0.25*change)
    return Score(total, mae, overshoot, undershoot, steady, ripple, change, capped,
                 settling(tu, eu), settling(td, ed), False)


class Models:
    def __init__(self, trials):
        self.x = unit([row["gains"] for row in trials])
        y = np.log([row["score"] for row in trials])
        self.y_scale = max(float(np.std(y)), 1e-10)
        shape = ConstantKernel(1., (0.04, 20.)) * Matern(
            np.full(3, 0.38), length_scale_bounds=(0.08, 2.5), nu=2.5)
        self.objective = GaussianProcessRegressor(shape + WhiteKernel(0.04, (1e-4, 0.8)),
                                                   normalize_y=True, alpha=1e-8)
        with warnings.catch_warnings(), threadpool_limits(limits=1):
            warnings.simplefilter("ignore", ConvergenceWarning)
            self.objective.fit(self.x, y)
            # Leave-one-out residuals avoid labelling interpolation as repeatability.
            inverse_chol = solve_triangular(self.objective.L_, np.eye(len(y)), lower=True)
            precision_diag = np.sum(inverse_chol**2, axis=0)
            error = self.objective.alpha_ / precision_diag * self.y_scale
            log_noise = np.log(np.maximum(0.015, np.abs(error)*np.sqrt(np.pi/2)))
            self.noise = GaussianProcessRegressor(shape + WhiteKernel(0.15, (0.02, 2)),
                                                  normalize_y=True, alpha=1e-6)
            self.noise.fit(self.x, log_noise)
        self.white_var = self.objective.kernel_.k2.noise_level * self.y_scale**2

    def predict(self, x):
        with threadpool_limits(limits=1):
            mu, total_std = self.objective.predict(x, return_std=True)
            latent = np.sqrt(np.maximum(total_std**2-self.white_var, 1e-10))
            noise = np.clip(np.exp(self.noise.predict(x)), 0.015, 3.)
        return mu, latent, noise

    def next(self, seed, reference=None):
        random = np.random.default_rng(seed)
        x = random.random((3000, 3))
        if reference is not None:
            x[:1500] = np.clip(unit(reference) + random.normal(0, 0.16, (1500, 3)), 0, 1)
        distant = np.min(np.linalg.norm(x[:, None, :]-self.x[None, :, :], axis=2), axis=1) > 0.025
        m, s, n = self.predict(x)
        tried, spread, _ = self.predict(self.x)
        incumbent = np.min(tried+0.5*spread)
        z = (incumbent-m)/s
        ei = (incumbent-m)*norm.cdf(z) + s*norm.pdf(z)
        aei = ei*(1-n/np.hypot(s, n))
        return tuple(gains_at(x[np.argmax(np.where(distant, aei, -1))]))

    def shortlist(self, trials, candidates=None):
        m, _, n = self.predict(self.x)
        chosen = []
        for i in np.argsort(m+n):
            if trials[i].get("fault") or (candidates is not None and trials[i]["group"] not in candidates):
                continue
            if all(np.linalg.norm(self.x[i]-self.x[j]) > 0.12 for j in chosen):
                chosen.append(i)
            if len(chosen) == 3:
                break
        return [tuple(trials[i]["gains"]) for i in chosen]

    def map(self, ki, points=38):
        axis = np.linspace(0, 1, points)
        kp, kd = np.meshgrid(axis, axis, indexing="ij")
        fixed = np.clip((np.log(ki)-LOG_LOW[1])/LOG_SPAN[1], 0, 1)
        x = np.c_[kp.ravel(), np.full(kp.size, fixed), kd.ravel()]
        m, _, n = self.predict(x)
        return np.exp(m.reshape(kp.shape)), n.reshape(kp.shape)
