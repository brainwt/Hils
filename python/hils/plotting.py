"""결과 그림 (matplotlib). 이중 y축 없이 small multiples, 고정 순서 범주색."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .state_machine import STATE_NAMES  # noqa: E402

C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"

plt.rcParams.update({
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 1.4, "font.size": 9, "legend.frameon": False,
    "figure.dpi": 110,
})


def _ma(v, n=60):
    v = np.asarray(v, float)
    if len(v) < n:
        return v
    c = np.cumsum(np.insert(v, 0, 0.0))
    out = np.empty_like(v)
    out[n - 1:] = (c[n:] - c[:-n]) / n
    out[:n - 1] = c[1:n] / np.arange(1, n)
    return out


def _shade_states(ax, t, state, x):
    """RUN 이외 구간(데이터 무효)을 옅은 회색으로 표시."""
    bad = np.asarray(state) != 3
    if bad.any():
        ax.fill_between(x, 0, 1, where=bad, transform=ax.get_xaxis_transform(), color="#9c9a92",
                        alpha=0.15, lw=0, step="post", label="not RUN (invalid)")


def plot_timeseries(log, path, title, xunit="h"):
    t = np.asarray(log["t"], float)
    x = t / 3600 if xunit == "h" else t / 60
    xl = "time [h]" if xunit == "h" else "time [min]"
    fig, ax = plt.subplots(4, 1, figsize=(9, 8.6), sharex=True,
                           gridspec_kw={"height_ratios": [3, 2, 2, 1.2]})
    a = ax[0]
    a.plot(x, _ma(log["Q_HP"]) / 1e3, color=C[2], lw=1.2, label="Q_HP heat pump (60 s mean)")
    a.plot(x, _ma(log["Q_load_meas"]) / 1e3, color=C[1], lw=1.2, label="Q_load_meas chamber (60 s mean)")
    a.plot(x, np.asarray(log["Q_target"]) / 1e3, color=C[0], lw=1.6, ls=(0, (4, 2)),
           label="Q_target virtual building", zorder=5)
    _shade_states(a, t, log["state"], x)
    a.set_ylabel("heat rate [kW]")
    a.legend(loc="upper right", ncol=2, fontsize=8)
    a.set_title(title, loc="left", color=INK, fontsize=11)
    a = ax[1]
    e = np.asarray(log["Q_ref"]) - np.asarray(log["Q_load_meas"])
    a.plot(x, e, color=C[0], lw=0.8)
    a.axhline(0, color=INK2, lw=0.6)
    a.set_ylabel("realization error\nQ_ref - Q_meas [W]")
    lim = max(200, np.percentile(np.abs(e), 99.5) * 1.2)
    a.set_ylim(-lim, lim)
    a = ax[2]
    a.plot(x, log["T_indoor"], color=C[0], label="T_indoor (chamber)")
    a.set_ylabel("T_indoor [°C]")
    a = ax[3]
    a.step(x, log["state"], where="post", color=C[6])
    a.set_yticks(range(6))
    a.set_yticklabels(STATE_NAMES, fontsize=7)
    a.set_ylim(-0.5, 5.5)
    a.set_xlabel(xl)
    fig.align_ylabels(ax)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_delay_sweep(rows, path):
    """rows: [(delay, comp:bool, rmse, energy_err, safe_stop_s)]"""
    d = sorted({r[0] for r in rows})
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    for i, (comp, lab) in enumerate([(False, "fixed PI gains"), (True, "delay-aware PI (SIMC)")]):
        r = sorted([x for x in rows if x[1] == comp])
        rm = [x[2] for x in r]
        ax[0].plot(d, rm, marker="o", ms=5, color=C[i], label=lab)
        ax[1].plot(d, [x[4] / 60 for x in r], marker="o", ms=5, color=C[i], label=lab)
        ax[0].annotate(f"{rm[-1]:.0f} W", (d[-1], rm[-1]), textcoords="offset points",
                       xytext=(4, 0), va="center", fontsize=8, color=INK)
    ax[0].set_xlabel("PLC command/ack delay [s]")
    ax[0].set_ylabel("RMSE Q_ref - Q_meas, all enabled samples [W]")
    ax[0].legend(fontsize=8)
    ax[1].set_xlabel("PLC command/ack delay [s]")
    ax[1].set_ylabel("time outside RUN after start-up [min]")
    ax[0].set_title("Realization error vs delay", loc="left", fontsize=10)
    ax[1].set_title("Lost test time (STABILIZING / SAFE_STOP)", loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
