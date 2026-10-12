"""
Slider-crank mechanism: position, velocity and acceleration analysis
by vector loop closure, verified by numerical differentiation.

Geometry (crank pivot O at the origin, slider moves along a line y = e):
    r  : crank length O-A
    l  : connecting-rod length A-B
    e  : slider offset (0 = in-line slider crank)
    theta : crank angle measured from +x axis (CCW positive)
    phi   : connecting-rod angle measured from +x axis

Loop closure:  r*e^(i*theta) + l*e^(i*phi) = x + i*e
"""
import numpy as np
import matplotlib.pyplot as plt

# ---------------- Parameters ----------------
r = 0.050        # crank length, m
l = 0.200        # connecting rod length, m
e = 0.0          # slider offset, m
omega2 = 100.0   # crank angular velocity, rad/s (CCW)  ~ 955 rpm
alpha2 = 0.0     # crank angular acceleration, rad/s^2


def analyse(theta, r=r, l=l, e=e, w2=omega2, a2=alpha2):
    """Return phi, x, omega3, v_B, alpha3, a_B for crank angle(s) theta (rad)."""
    # Position
    sphi = (e - r * np.sin(theta)) / l
    phi = np.arcsin(sphi)                     # open (normal) assembly
    cphi = np.cos(phi)
    x = r * np.cos(theta) + l * cphi

    # Velocity (differentiate loop equations once)
    w3 = -r * w2 * np.cos(theta) / (l * cphi)
    vB = -r * w2 * np.sin(theta) - l * w3 * sphi

    # Acceleration (differentiate twice)
    a3 = (r * w2**2 * np.sin(theta) - r * a2 * np.cos(theta)
          + l * w3**2 * sphi) / (l * cphi)
    aB = (-r * a2 * np.sin(theta) - r * w2**2 * np.cos(theta)
          - l * a3 * sphi - l * w3**2 * cphi)
    return phi, x, w3, vB, a3, aB


if __name__ == "__main__":
    # ---------- Worked example at theta = 45 deg ----------
    th = np.radians(45)
    phi, x, w3, vB, a3, aB = analyse(th)
    vA = r * omega2
    aA = r * omega2**2
    vBA = l * abs(w3)
    print("=== Worked example: theta = 45 deg ===")
    print(f"Rod angle phi          = {np.degrees(phi):9.3f} deg")
    print(f"Slider position x_B    = {x*1000:9.3f} mm")
    print(f"|v_A| = r*w2           = {vA:9.3f} m/s")
    print(f"Rod angular vel w3     = {w3:9.3f} rad/s")
    print(f"|v_B/A| = l*|w3|       = {vBA:9.3f} m/s")
    print(f"Slider velocity v_B    = {vB:9.3f} m/s")
    print(f"|a_A| = r*w2^2         = {aA:9.3f} m/s^2")
    print(f"Rod centripetal l*w3^2 = {l*w3**2:9.3f} m/s^2")
    print(f"Rod angular acc a3     = {a3:9.3f} rad/s^2")
    print(f"Rod tangential l*a3    = {l*a3:9.3f} m/s^2")
    print(f"Slider accel a_B       = {aB:9.3f} m/s^2")
    n = l / r
    aB_approx = -r * omega2**2 * (np.cos(th) + np.cos(2*th) / n)
    print(f"Approx a_B (2-term)    = {aB_approx:9.3f} m/s^2")

    # ---------- Full revolution ----------
    N = 3601
    theta = np.linspace(0, 2*np.pi, N)
    phi, x, w3, vB, a3, aB = analyse(theta)

    # Numerical check: differentiate x(t) with t = theta/omega2
    dt = (theta[1] - theta[0]) / omega2
    v_num = np.gradient(x, dt)
    a_num = np.gradient(v_num, dt)
    inner = slice(5, -5)
    print("\n=== Verification vs. numerical differentiation ===")
    print(f"max |v_analytic - v_numeric| = {np.max(np.abs(vB - v_num)[inner]):.2e} m/s")
    print(f"max |a_analytic - a_numeric| = {np.max(np.abs(aB - a_num)[inner]):.2e} m/s^2")

    print("\n=== Extremes over one revolution ===")
    print(f"Stroke = {(x.max()-x.min())*1000:.2f} mm")
    i = np.argmax(np.abs(vB))
    print(f"max |v_B| = {abs(vB[i]):.3f} m/s at theta = {np.degrees(theta[i]):.1f} deg")
    print(f"a_B at TDC (0 deg)   = {aB[0]:.2f} m/s^2")
    print(f"a_B at BDC (180 deg) = {aB[N//2]:.2f} m/s^2")
    print(f"max |w3| = {np.max(np.abs(w3)):.3f} rad/s, max |a3| = {np.max(np.abs(a3)):.1f} rad/s^2")

    # ---------- Plots ----------
    deg = np.degrees(theta)
    fig, ax = plt.subplots(2, 3, figsize=(15, 8.5))
    fig.suptitle(f"Slider-crank analysis  (r = {r*1000:.0f} mm, l = {l*1000:.0f} mm, "
                 f"e = {e*1000:.0f} mm, ω₂ = {omega2:.0f} rad/s CCW)", fontsize=13)

    # Mechanism sketch at 45 deg with velocity vectors
    a0 = ax[0, 0]
    t0 = np.radians(45)
    p0, x0, w30, vB0, *_ = analyse(t0)
    A = np.array([r*np.cos(t0), r*np.sin(t0)])
    B = np.array([x0, e])
    a0.plot([0, A[0]], [0, A[1]], 'o-', lw=4, color='tab:blue', label='Crank OA')
    a0.plot([A[0], B[0]], [A[1], B[1]], 'o-', lw=4, color='tab:orange', label='Rod AB')
    a0.add_patch(plt.Rectangle((B[0]-0.015, e-0.01), 0.03, 0.02, color='gray'))
    a0.axhline(e - 0.012, color='k', lw=1); a0.axhline(e + 0.012, color='k', lw=1)
    a0.plot(r*np.cos(theta), r*np.sin(theta), 'k:', lw=0.8)
    s = 0.01  # arrow scale m per (m/s)
    vA_vec = r*omega2*np.array([-np.sin(t0), np.cos(t0)])
    a0.arrow(*A, *(vA_vec*s), width=0.0015, color='tab:green', label='v_A')
    a0.arrow(*B, vB0*s, 0, width=0.0015, color='tab:red', label='v_B')
    a0.text(0.004, -0.012, 'O'); a0.text(A[0]+0.004, A[1], 'A'); a0.text(B[0], B[1]+0.016, 'B')
    a0.set_aspect('equal'); a0.set_title('Configuration at θ = 45°'); a0.legend(fontsize=7, loc='lower left', ncol=2)
    a0.set_xlim(-0.08, 0.28); a0.set_ylim(-0.08, 0.09); a0.set_xlabel('m')

    ax[0, 1].plot(deg, x*1000); ax[0, 1].set_title('Slider position $x_B$'); ax[0, 1].set_ylabel('mm')
    ax[0, 2].plot(deg, vB, label='analytic'); ax[0, 2].plot(deg[::60], v_num[::60], 'k.', ms=4, label='numerical d/dt')
    ax[0, 2].set_title('Slider velocity $v_B$'); ax[0, 2].set_ylabel('m/s'); ax[0, 2].legend(fontsize=8)
    ax[1, 0].plot(deg, aB, label='exact')
    ax[1, 0].plot(deg, -r*omega2**2*(np.cos(theta) + (r/l)*np.cos(2*theta)), '--', label='2-term approx.')
    ax[1, 0].plot(deg[60:-60:60], a_num[60:-60:60], 'k.', ms=4, label='numerical d²/dt²')
    ax[1, 0].set_title('Slider acceleration $a_B$'); ax[1, 0].set_ylabel('m/s²'); ax[1, 0].legend(fontsize=8)
    ax[1, 1].plot(deg, w3, color='tab:orange'); ax[1, 1].set_title('Rod angular velocity $ω_3$'); ax[1, 1].set_ylabel('rad/s')
    ax[1, 2].plot(deg, a3, color='tab:orange'); ax[1, 2].set_title('Rod angular acceleration $α_3$'); ax[1, 2].set_ylabel('rad/s²')
    for a in ax.flat[1:]:
        a.set_xlabel('Crank angle θ (deg)'); a.set_xticks(range(0, 361, 45)); a.grid(alpha=0.3)
        a.axvline(45, color='gray', ls=':', lw=1)
    plt.tight_layout()
    plt.savefig('slider_crank_analysis.png', dpi=130)
    print("\nSaved slider_crank_analysis.png")
