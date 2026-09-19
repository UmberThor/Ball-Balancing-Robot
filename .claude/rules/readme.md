---
paths:
  - "README.md"
  - "docs/**"
---

# README and docs

## README style

Precise, brief, impersonal technical prose in full sentences (not narrative, not telegraphic bullet fragments); lists only for things like the parts list. Symbols for vectors use `\mathbf`, subscripts without commas (`B_{ix}`). Sections should not rely on later sections where avoidable. GitHub math rendering: avoid `\,` `\;` `\!` `\{` `\}` (use `\ `, `\quad`, `\lbrace`/`\rbrace`), leave a blank line before `$$`, don't start a math line with `+ `/`- `, and use HTML entities rather than LaTeX inside HTML captions.

## Rendered figures

- `docs/ball_model.png` (README §8.1) is rendered from `docs/ball_model.tex`: `pdflatex ball_model.tex && pdftoppm -png -r 250 -singlefile ball_model.pdf ball_model`.
