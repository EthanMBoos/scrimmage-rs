# Documentation site options

Research notes, 2026-09-24, for publishing student-facing docs to GitHub Pages at
`https://ethanmboos.github.io/scrimmage-rs/`.

**Status:** mdBook was chosen and is set up in [`book/`](../book/), with the
styling below and the deploy workflow in
[`.github/workflows/book.yml`](../.github/workflows/book.yml). Publishing needs
one manual step: Settings → Pages → Source: **GitHub Actions**. Preview locally
with `mdbook serve book --open` (install with `cargo install mdbook --locked`).
The rest of this page is the research behind that choice.

Two candidates were reviewed: [mdBook](https://rust-lang.github.io/mdBook/)
and [Starlight](https://starlight.astro.build/). Both use plain Markdown and
deploy as static sites to GitHub Pages.

## Recommendation

For a research and teaching repo that values simplicity, lean toward **mdBook**
with the styling below:

- no Node toolchain; students only need `cargo install mdbook`;
- the same Markdown renders correctly both on GitHub and on the site.

Choose **Starlight** instead if a polished landing page and a modern component
set matter more than adding a second toolchain.

Either way, keep `docs/` as internal engineering notes (evidence, TODO,
reference notes, style guide, and the copied C++ guides). `AGENTS.md` points
agents at those files by exact name, and the copied C++ guides must stay
unchanged. Put student-facing pages in a separate book or site folder.

## mdBook

### Public sentiment

- **Liked:** simplicity. Typical comments: "super easy to install... just
  `cargo install mdbook`" and "I love how simple it is." It is the Rust-world
  default: the Rust Book, the Cargo book, and the rustc dev guide use it.
- **Criticized:**
  - it looks plain and "book-like" rather than like a product docs site;
  - it had no on-page table of contents;
  - code highlighting (highlight.js) and math (MathJax) run in the browser, which
    some call wasteful;
  - no doc versioning, and limited extensibility.
- **Alternatives people suggest:** Material for MkDocs ("so much more built in")
  and Docusaurus. Starlight has become the usual "what about this one?" answer
  in threads that used to recommend Docusaurus.
- **Recent changes:** mdBook 0.5 (current release 0.5.4) addressed the two
  biggest complaints:
  - the sidebar now lists the current page's headings;
  - GitHub-style callouts (`> [!NOTE]`, `> [!TIP]`, `> [!WARNING]`) are built in.

  Much of the older criticism predates 0.5.

### Why it fits this repo

- `> [!NOTE]` callouts and relative `.md` links render correctly both on GitHub
  and in mdBook, so pages read the same in the repo and on the site.
- It is a single Rust binary with no `package.json` or `node_modules`.
- Links that leave the book folder, such as `../crates/...`, still break on the
  published site. Student-facing pages should link to source files with full
  `https://github.com/EthanMBoos/scrimmage-rs/blob/main/...` URLs.

### Making the default style look better

Most of the dated look comes from:

- the default Open Sans font;
- the narrow 750px text column;
- the five-theme picker (Light, Rust, Coal, Navy, Ayu).

A small `book.toml` and about 30 lines of CSS fix these. The CSS variable names
below were checked against mdBook's current `variables.css`.

`book.toml`:

```toml
[book]
title = "scrimmage-rs"
src = "src"

[output.html]
site-url = "/scrimmage-rs/"
git-repository-url = "https://github.com/EthanMBoos/scrimmage-rs"
edit-url-template = "https://github.com/EthanMBoos/scrimmage-rs/edit/main/book/{path}"
default-theme = "light"
preferred-dark-theme = "coal"      # follows the reader's OS dark-mode setting
additional-css = ["theme/custom.css"]
smart-punctuation = true

[output.html.fold]
enable = true                      # collapsible sidebar sections
level = 1
```

`theme/custom.css`:

```css
:root {
  --content-max-width: 860px;          /* default 750px feels cramped */
  --sidebar-target-width: 280px;
  --mono-font: "JetBrains Mono", ui-monospace, Menlo, Consolas, monospace;
}
html { font-family: Inter, system-ui, -apple-system, "Segoe UI", sans-serif; }
.content { line-height: 1.7; }
.content h1, .content h2, .content h3 { font-weight: 650; letter-spacing: -0.01em; }
.content h2 { margin-top: 2.2em; padding-bottom: .3em; border-bottom: 1px solid var(--table-border-color); }

/* One accent color instead of the default link blue */
.light, .rust { --links: #2f6fdb; --sidebar-active: #2f6fdb; }
.coal, .navy, .ayu { --links: #7aa7ff; --sidebar-active: #7aa7ff; }

/* Softer code and tables */
.content pre { border-radius: 8px; }
.content :not(pre) > code { padding: .1em .35em; border-radius: 4px; }
.content table { border-radius: 8px; overflow: hidden; }
```

Optional extras:

- To load real web fonts, add a `theme/head.hbs` with a Google Fonts `<link>`.
  Otherwise the system fonts above are used.
- Hide the theme picker and keep only light/dark; the Rust and Ayu themes are
  what make sites look dated.
- Add a small team-colored logo to the menu bar.
- For a ready-made look, [catppuccin/mdBook](https://github.com/catppuccin/mdbook)
  is the most widely used community theme.

CSS cannot provide Starlight's landing-page layout or its component library.
The styled version has not been built yet; build a couple of pages and
screenshot them before deciding.

### Proposed layout

```
book/
  book.toml
  theme/custom.css
  src/
    SUMMARY.md                 # sidebar order
    introduction.md            # what scrimmage-rs is, quick start
    tutorial/first-autonomy.md
    concepts/coordinate-frames.md
    concepts/belief-vs-truth.md
    guides/writing-plugins.md  # adapted from docs/RUST_PLUGINS.md
.github/workflows/book.yml     # install mdbook, build, deploy to Pages
```

## Starlight

Starlight is a docs theme built on the Astro framework.

### What it provides

- **Setup:** `npm create astro@latest -- --template starlight`, or
  `npx astro add starlight` inside an existing Astro project.
- **Pages:** every `.md` file under `src/content/docs/` becomes a page. Each page
  needs YAML frontmatter with at least a `title`.
- **Built in:** folder-generated sidebar, full-text search (Pagefind) that works
  on static hosting, dark mode, "Edit this page" links, and callouts
  (`:::note`, `:::tip`, `:::caution`, `:::danger`).
- **Code blocks** (via Expressive Code): titles, line highlighting, and diff
  markers.
- **Requirements:** Node 22.12 or newer, even-numbered versions only.

### GitHub Pages deployment

1. In `astro.config.mjs`, set `site: 'https://ethanmboos.github.io'` and
   `base: '/scrimmage-rs'`.
2. Add `.github/workflows/deploy.yml` using `withastro/action@v6` and
   `actions/deploy-pages@v5`, triggered on pushes to `main`. If the site lives
   in a subfolder such as `site/`, pass that folder as the action's `path`.
3. On GitHub, under Settings → Pages, choose **GitHub Actions** as the source.

### Problems for this repo

- **Links in page content don't get the `/scrimmage-rs` prefix.** Astro only
  adds it to links it generates itself, such as the sidebar. Fixes:
  - use relative links;
  - or add [starlight-base-path](https://github.com/andriygm/starlight-base-path).

  Also add [starlight-links-validator](https://github.com/HiDeoo/starlight-links-validator)
  so the build fails on broken links.
- **Links between `.md` files are not converted.** A link like
  `[x](RUST_PLUGINS.md)` does not point to the right page without
  [astro-rehype-relative-markdown-links](https://github.com/vernak2539/astro-rehype-relative-markdown-links)
  or rewriting the links.
- **About 57 links in `docs/` point into the code** (`../crates`, `../reference`,
  `../runs`, `../../scrimmage`). They would need to become full GitHub URLs.
  Links into `runs/` and the sibling `../scrimmage` repo cannot work on any
  site, because those files are not on GitHub.
- **It adds Node** (`package.json`, `node_modules`, a lockfile) to a Rust
  project.

## Content worth bringing over from the C++ site

The old SCRIMMAGE Sphinx docs (`../scrimmage/docs/source`) are uneven: the
concept pages hold up, but the step-by-step tutorials are stale even for C++.
Rewrite these for Rust rather than porting them:

1. **Follow-the-nearest-opponent autonomy** (from "Create an Autonomy Plugin").
   About 30 lines covering contacts, team IDs, controller outputs, XML
   parameters, and editing and running a mission. It maps onto
   `contacts_truth` and the `desired_heading` / `desired_altitude` outputs.
2. **Coordinate frames.**
   - ENU axes; yaw 0 points east, not north as in GPS headings.
   - Add this port's traps: positive pitch points the nose down, SimpleAircraft's
     model roll sign is flipped, and FixedWing6DOF works internally in a
     forward/right/down frame.
   - The C++ page's FixedWing6DOF snippet no longer matches the C++ code.
3. **Belief vs truth**, from "Design Principles": why the estimated and true
   state are separate, and the pattern of a single sensor that fuses readings
   into the estimate (for example an INS).
4. **Publish/subscribe ideas.**
   - Messages are delivered on the next step, which keeps runs deterministic.
   - LocalNetwork is for data that stays on one vehicle; GlobalNetwork is for
     simulation-wide data.
   - To expose local data, republish it globally under an entity-ID topic.
5. **Capture-the-flag scenario design.**
   - Game rules are interaction plugins that publish events.
   - Metrics plugins turn those events into scores.
   - Autonomy and motion plugins stay unaware of the game.
6. **Mission-level test**: run a mission and assert on `summary.csv`.

Skip these:

- project creation, `generate-plugin.sh`, CMake, and `REGISTER_PLUGIN`, which
  rely on the excluded runtime plugin loading;
- the stale C++ motion, controller, and metrics walkthroughs;
- runtime parameters and protobuf `GenerateEntity` spawning (excluded features);
- install, viewer, logging, environment variables, and the XML tag reference;
- ROS, AirSim, terrain, GPU, OpenGrid, and optimization pages.

The "Multiple Local Runs" tutorial describes the main research workflow: many
randomized runs comparing algorithms by team score. This port has no batch
runner yet, so that is a feature question for `docs/TODO.md`, not a docs task.

## Sources

- [mdBook HTML renderer options](https://rust-lang.github.io/mdBook/format/configuration/renderers.html)
- [mdBook CHANGELOG](https://github.com/rust-lang/mdBook/blob/main/CHANGELOG.md)
- [mdBook 0.5.4 on docs.rs](https://docs.rs/crate/mdbook/latest)
- [mdBook issue #1523: in-page TOC](https://github.com/rust-lang/mdBook/issues/1523)
- [mdbook-pagetoc](https://github.com/slowsage/mdbook-pagetoc)
- [catppuccin/mdBook](https://github.com/catppuccin/mdbook)
- [HN: MdBook discussion (2023)](https://news.ycombinator.com/item?id=36528984)
- [HN: mdBook discussion (2020)](https://news.ycombinator.com/item?id=22854272)
- [Rust forum: doubled TOC entries](https://users.rust-lang.org/t/state-of-the-ugly-doubled-toc-entries-of-mdbook/139529)
- [Starlight manual setup](https://starlight.astro.build/manual-setup/)
- [Starlight authoring content](https://starlight.astro.build/guides/authoring-content/)
- [Starlight configuration reference](https://starlight.astro.build/reference/configuration/)
- [Astro: deploy to GitHub Pages](https://docs.astro.build/en/guides/deploy/github/)
- [Astro install requirements](https://docs.astro.build/en/install-and-setup/)
- [starlight-base-path](https://github.com/andriygm/starlight-base-path)
- [starlight-links-validator](https://github.com/HiDeoo/starlight-links-validator)
- [astro-rehype-relative-markdown-links](https://github.com/vernak2539/astro-rehype-relative-markdown-links)
- [LogRocket: Starlight vs Docusaurus](https://blog.logrocket.com/starlight-vs-docusaurus-building-documentation/)
- [Docsio: Starlight review 2026](https://docsio.co/blog/starlight-docs)
