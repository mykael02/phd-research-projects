"""Derive a nominal LQR from the nonlinear model and export both runtimes' config."""
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
from scipy.linalg import solve_continuous_are
from balance import Parameters, derivative


def design():
    p, eps = Parameters(), 1e-6
    basis = np.eye(4)
    a = np.column_stack([(np.array(derivative(eps*col,0,p=p))-np.array(derivative(-eps*col,0,p=p)))/(2*eps) for col in basis])
    b = ((np.array(derivative(np.zeros(4),eps,p=p))-np.array(derivative(np.zeros(4),-eps,p=p)))/(2*eps)).reshape(4,1)
    q, r = np.diag([12, 2, 120, 2]), np.array([[1.0]])
    solution = solve_continuous_are(a,b,q,r)
    gain = np.linalg.solve(r,b.T@solution)
    poles = np.linalg.eigvals(a-b@gain)
    residual = a.T@solution+solution@a-solution@b@np.linalg.solve(r,b.T)@solution+q
    assert np.max(poles.real)<0
    assert np.linalg.norm(residual)<1e-7
    return {"parameters":asdict(p),"dt":.005,"fall_angle_degrees":45,
            "lqr_gain":gain[0].tolist(),"lqr_q":q.diagonal().tolist(),"lqr_r":1.0,
            "linear_a":a.tolist(),"linear_b":b.tolist(),
            "closed_loop_poles":[[z.real,z.imag] for z in poles],
            "pid":{"kp":14.0,"ki":0.4,"kd":2.0,"kx":2.0,"kv":3.0,"integral_limit":.25}}


if __name__ == "__main__":
    config = design()
    root=Path(__file__).resolve().parent
    (root/"config.json").write_text(json.dumps(config,indent=2)+"\n",encoding="utf-8")
    (root/"config.js").write_text("const BALANCE_CONFIG = "+json.dumps(config)+";\nif (typeof module !== 'undefined') module.exports = BALANCE_CONFIG;\n",encoding="utf-8")
    print("LQR gain:",config["lqr_gain"])
    print("Closed-loop poles:",config["closed_loop_poles"])
