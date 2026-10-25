# Mechanics Labs

Interactive, browser-based models for teaching theory of machines and mechanical vibrations. Everything is static HTML and JavaScript, with no build step and nothing to install. It is hosted free with GitHub Pages.

| Lab | Folder | Topic |
|---|---|---|
| Slider-Crank Lab | [`slider-crank/`](slider-crank/) | Position, velocity and acceleration analysis by loop closure; velocity and acceleration polygons |
| Quick Return Lab | [`quick-return/`](quick-return/) | Crank and slotted lever; time ratio; Coriolis acceleration |
| Spur Gear Mesh Lab | [`gears/`](gears/) | Involute spur gears and rack; contact ratio; interference; sliding; tooth load and contact stress |
| Turning-Moment & Flywheel Lab | [`flywheel/`](flywheel/) | Gas and inertia forces; turning-moment diagrams; fluctuation of energy; flywheel design |
| Friction Clutch Lab | [`clutch/`](clutch/) | Plate clutches; uniform pressure and uniform wear; engagement, slip, heat and stalling; parts and materials |
| BME 2102 interactive notes | [`bme2102/`](bme2102/) | All seven chapters of the course notes (introduction, simple mechanisms, velocity and acceleration, belt drives, toothed gearing, cams, balancing) as slide decks with 3D models (three.js, in `vendor/`), animations, live worked examples and self-check questions. Roboto is bundled in `bme2102/assets/fonts/` |
| Four-Cylinder Engine Lab | [`engine/`](engine/) | Mechanics of Machines II with the cut-out engines: slider-crank kinematics, cylinder pressure, turning moment and flywheel, primary and secondary balancing; printable lab sheet and degree wheel (`engine/sheet.html`) |
| Spring–Mass–Damper Lab | [`vibration/`](vibration/) | Vertical SDOF system; free, forced and base-excited vibration |
| Tacoma Narrows Flutter Lab | [`tacoma-narrows/`](tacoma-narrows/) | 1940 collapse; self-excited flutter versus vortex resonance |
| Solar Installer Training | [separate repository](https://github.com/olomo-umu/solar-installer-training) | Course for solar PV installers: safety, design, installation, commissioning, maintenance; simulators, quizzes, certificate |
| Uganda Electricity Atlas | [separate repository](https://github.com/olomo-umu/uganda-electricity-atlas) | Uganda’s power system: load flow, frequency, blackouts, scenarios, losses, access and open data |

`python/slider_crank.py` is a NumPy and Matplotlib version of the slider-crank analysis.

## Publish with GitHub Pages

1. Create a new **public** repository, for example `mechanics-labs`.
2. Upload the *contents* of this folder to the repository root (drag and drop on **Add file → Upload files**), then commit.
3. Open **Settings → Pages**. Under **Build and deployment**, choose **Deploy from a branch**, branch **main**, folder **/ (root)**, and save.
4. After a minute or two the site is live at `https://<your-username>.github.io/mechanics-labs/`.

Share that link with students. Each lab also has its own address, for example `https://<your-username>.github.io/mechanics-labs/vibration/`.

## Adding a lab

Put a new folder containing an `index.html` beside the others, then add a card for it in the root `index.html`. The empty `.nojekyll` file tells GitHub Pages to serve the files exactly as they are.

## References

Billah, K. Y., & Scanlan, R. H. (1991). Resonance, Tacoma Narrows bridge failure, and undergraduate physics textbooks. *American Journal of Physics, 59*(2), 118–124. https://doi.org/10.1119/1.16590

Green, D., & Unruh, W. G. (2004). *Tacoma bridge failure: A physical model* (arXiv:physics/0408101). arXiv. https://arxiv.org/abs/physics/0408101

Heywood, J. B. (2018). *Internal combustion engine fundamentals* (2nd ed.). McGraw-Hill Education.

Inman, D. J. (2014). *Engineering vibration* (4th ed.). Pearson.

Khurmi, R. S., & Gupta, J. K. (2005). *Theory of machines* (14th ed.). S. Chand.

Norton, R. L. (2020). *Design of machinery: An introduction to the synthesis and analysis of mechanisms and machines* (6th ed.). McGraw-Hill Education.

Olson, D. W., Wolf, S. F., & Hook, J. M. (2015). The Tacoma Narrows Bridge collapse. *Physics Today, 68*(11), 64–65. https://doi.org/10.1063/PT.3.2991

Rao, S. S. (2017). *Mechanical vibrations* (6th ed.). Pearson.

Rattan, S. S. (2009). *Theory of machines* (3rd ed.). Tata McGraw-Hill.

Uicker, J. J., Pennock, G. R., & Shigley, J. E. (2017). *Theory of machines and mechanisms* (5th ed.). Oxford University Press.
