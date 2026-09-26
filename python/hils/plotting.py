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


def _shade(ax, state, x):
    bad = np.asarray(state) != 3
    if bad.any():
        ax.fill_between(x, 0, 1, where=bad, transform=ax.get_xaxis_transform(), color="#9c9a92",
                        alpha=0.15, lw=0, step="post", label="not RUN (invalid)")


def plot_timeseries(log, path, title, xunit="h", ref=None):
    """ref: 이상적 결합(기준해석) 로그 - 존 온도 비교선."""
    t = np.asarray(log["t"], float)
    div = 3600 if xunit == "h" else 60
    x = t / div
    fig, ax = plt.subplots(5, 1, figsize=(9, 10.5), sharex=True,
                           gridspec_kw={"height_ratios": [2.4, 2, 2, 1.6, 1.1]})
    a = ax[0]
    if ref is not None:
        a.plot(np.asarray(ref["t"]) / div, ref["T_z"], color=C[3], lw=1.2, ls=(0, (1, 1.5)),
               label="T_zone, ideal coupling (reference)")
    a.plot(x, log["T_z"], color=C[0], lw=1.6, label="T_zone virtual (simulation output)")
    a.plot(x, _ma(log["T_return"]), color=C[1], lw=1.1, label="T_return chamber (measured)")
    a.plot(x, _ma(log["T_supply"]), color=C[2], lw=1.1, label="T_supply indoor unit (measured)")
    _shade(a, log["state"], x)
    a.set_ylabel("temperature [°C]")
    a.legend(loc="best", ncol=2, fontsize=7.5)
    a.set_title(title, loc="left", color=INK, fontsize=11)
    a = ax[1]
    a.plot(x, log["RH_z"], color=C[0], lw=1.6, label="RH_zone virtual")
    a.plot(x, _ma(log["RH_return"]), color=C[1], lw=1.1, label="RH_return chamber")
    a.set_ylabel("relative humidity [%]")
    a.legend(loc="best", fontsize=7.5)
    a = ax[2]
    a.plot(x, _ma(log["Q_sens"]) / 1e3, color=C[0], label="sensible (60 s mean)")
    a.plot(x, _ma(log["Q_lat"]) / 1e3, color=C[1], label="latent (60 s mean)")
    a.axhline(0, color=INK2, lw=0.6)
    a.set_ylabel("heat to zone,\nair-enthalpy [kW]")
    a.legend(loc="best", fontsize=7.5)
    a = ax[3]
    eT = np.asarray(log["T_return"]) - np.asarray(log["T_ref"])
    a.plot(x, eT, color=C[6], lw=0.7)
    a.axhline(0, color=INK2, lw=0.6)
    a.set_ylabel("tracking error\nT_return − T_sp [K]")
    lim = max(0.3, float(np.percentile(np.abs(eT), 99.5)) * 1.2)
    a.set_ylim(-lim, lim)
    a = ax[4]
    a.step(x, log["state"], where="post", color=C[6])
    a.set_yticks(range(6))
    a.set_yticklabels(STATE_NAMES, fontsize=7)
    a.set_ylim(-0.5, 5.5)
    a.set_xlabel("time [h]" if xunit == "h" else "time [min]")
    fig.align_ylabels(ax)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_delay_sweep(rows, path):
    """rows: [dict(delay_s, rmse_Tz_vs_ideal_K, energy_err_pct)]"""
    d = [r["delay_s"] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.3))
    for a, key, lab, col in [(ax[0], "rmse_Tz_vs_ideal_K", "RMSE T_zone vs ideal coupling [K]", C[0]),
                             (ax[1], "energy_err_pct", "HP energy vs ideal coupling [%]", C[1])]:
        y = [r[key] for r in rows]
        a.plot(d, y, marker="o", ms=5, color=col)
        a.annotate(f"{y[-1]:.2f}", (d[-1], y[-1]), textcoords="offset points", xytext=(5, 0),
                   va="center", fontsize=8)
        a.set_xlabel("PLC command/ack delay [s]")
        a.set_ylabel(lab)
    ax[1].axhline(0, color=INK2, lw=0.6)
    ax[0].set_title("Zone temperature fidelity", loc="left", fontsize=10)
    ax[1].set_title("Energy fidelity", loc="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
