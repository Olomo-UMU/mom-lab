# Mechanics Labs

Interactive, browser-based models for teaching theory of machines and mechanical vibrations. Everything is static HTML and JavaScript, with no build step and nothing to install. It is hosted free with GitHub Pages.

| Lab | Folder | Topic |
|---|---|---|
| Slider-Crank Lab | [`slider-crank/`](slider-crank/) | Position, velocity and acceleration analysis by loop closure; velocity and acceleration polygons |
| Quick Return Lab | [`quick-return/`](quick-return/) | Crank and slotted lever; time ratio; Coriolis acceleration |
| Spring–Mass–Damper Lab | [`vibration/`](vibration/) | Vertical SDOF system; free, forced and base-excited vibration |
| Tacoma Narrows Flutter Lab | [`tacoma-narrows/`](tacoma-narrows/) | 1940 collapse; self-excited flutter versus vortex resonance |

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

Inman, D. J. (2014). *Engineering vibration* (4th ed.). Pearson.

Norton, R. L. (2020). *Design of machinery: An introduction to the synthesis and analysis of mechanisms and machines* (6th ed.). McGraw-Hill Education.

Olson, D. W., Wolf, S. F., & Hook, J. M. (2015). The Tacoma Narrows Bridge collapse. *Physics Today, 68*(11), 64–65. https://doi.org/10.1063/PT.3.2991

Rao, S. S. (2017). *Mechanical vibrations* (6th ed.). Pearson.

Uicker, J. J., Pennock, G. R., & Shigley, J. E. (2017). *Theory of machines and mechanisms* (5th ed.). Oxford University Press.
