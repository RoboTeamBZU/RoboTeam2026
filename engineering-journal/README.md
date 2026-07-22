# 📓 Engineering Journal

**RoboTeamBZU — Birzeit University, 🇵🇸 Palestine** · Bashar Ibrahim, Jinan Yousef, Basmala Assi

The engineering journal is our **design story told in versions**. Each version is one full iteration cycle — what we set out to do, what we built, what broke, what we learned, and what changed as a result. This is the evidence behind the trade-offs summarized in [Systems Engineering](../docs/Systems-Engineering.md).

## Timeline

```
V1  ──►  V2  ──►  V3  ──►  Final
baseline  clean    steady   repeatable
car       sensing  control  3-lap runs
```

| Version | Theme | Headline change | Details |
|---|---|---|---|
| [Version 1](./Version-1) | Baseline platform | Car-like chassis drives under gyro PID | drives but wanders & stalls |
| [Version 2](./Version-2) | Clean sensing & power | LiDAR to top deck, split power rails | no brownouts, clean scans |
| [Version 3](./Version-3) | Steady control | Hardware-PWM servo, motor kick pulse | no jitter, reliable launch |
| [Final Version](./Final-Version) | Repeatability | Start-line recalibration, centering, wind-up fix | consistent 3-lap runs |

## How to read each version
Every version folder has a `README.md` (the write-up) and an `images/` folder for the photos, CAD screenshots, and test captures that document that stage. Add images with clear filenames as noted in each `images/README.md`.

> 📌 For submission, export the whole journal to a single PDF and place it per [`docs/Engineering-Journal-PDF.md`](../docs/Engineering-Journal-PDF.md).
